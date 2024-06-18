
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "roadlib.h"
#include "gviewer.h"
#include "visualization.h"

#include <pcl/registration/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <fstream>


//** PHASE : Incremental mapping.
int main_phase_mapping(const SensorConfig & config, 
	map<double, string> camstamp, string raw_dir, string semantic_dir,
	const Trajectory & traj_ref,
	map<double, shared_ptr<RoadInstancePatchFrame>> & all_frames,
	RoadInstancePatchMap &road_map)
{
	gv::IPMProcesser ipm_processer(config.cam, gv::IPMType::NORMAL);


	std::cerr << "[ PHASE ] : Semantic mapping." << std::endl;

	// A pose window for smoothing.
	deque<pair<Eigen::Matrix3d, Eigen::Vector3d>> pose_history;
	Eigen::Vector3d last_att_ave;

	road_map.ref = traj_ref.ref;
	vector<vector<Vector3d>> vis_points_this_frame;
	vector<pair<Eigen::Matrix3d, Eigen::Vector3d>> old_poses;
	int frame_count = 0;

	for (auto image_iter = camstamp.begin(); image_iter != camstamp.end(); image_iter++)
	{	
		if (image_iter->first < config.t_start) continue;
		if (image_iter->first > config.t_end) break;

		frame_count++;

		Eigen::Matrix3d R_c1c0 = config.cam.cg.getR();

		// Get the latest IMU pose.
		auto this_pose = traj_ref.poses.lower_bound(image_iter->first - 0.001);
		if (fabs(this_pose->first - image_iter->first) > 0.001) continue; // Pose of this moment not found.

		pose_history.push_back(make_pair(this_pose->second.R, this_pose->second.t));
		while (pose_history.size() > config.pose_smooth_window)
		{
			pose_history.pop_front();
		}

		if (config.need_smooth)
		{
			// Transformation between the smoothed IMU frame and the latest IMU frame.
			Eigen::Vector3d att_ave(0, 0, 0);
			Eigen::Vector3d att_diff;
			Eigen::Matrix3d R_b_n;
			Eigen::Vector3d att_this;
			for (int i = 0; i < pose_history.size(); i++)
			{
				R_b_n = pose_history[i].first;
				att_this = m2att(R_b_n);
				att_ave += att_this;
				if (i == pose_history.size() - 1)
				{
					att_ave /= pose_history.size();
					att_diff = att_this - att_ave;
				}
			}
#ifdef DEBUG
			std::cerr << "pitch : " << att_ave(0) * 180 / M_PI << " " << att_this(0) * 180 / M_PI << " " << att_diff(0) * 180 / M_PI <<
				" " << (att_ave(0) - last_att_ave(0)) * 180 / M_PI << std::endl;
#endif
			if (fabs((att_ave(0) - last_att_ave(0)) *R2D) > config.large_slope_thresold / config.pose_smooth_window && pose_history.size()>5) // big road slope change detected
			{
				std::cerr << "[INFO] Large slope detected!" << std::endl;
				pose_history.clear();
				pose_history.push_back(make_pair(this_pose->second.R, this_pose->second.t));
				last_att_ave = att_this;
				int count_temp = 0;
				for (auto iter_frame = all_frames.rbegin(); iter_frame != all_frames.rend() && count_temp < 20; iter_frame++)
				{
					Vector3d ti0i1 = iter_frame->second->R.transpose()*(this_pose->second.t - iter_frame->second->t);
					std::cout << "[INFO] ti0i1 : " << ti0i1.transpose() << std::endl;
					if (ti0i1(1) < 13.5 && (ti0i1(0)) < 5)
					{
						road_map.ignore_frame_ids.insert(make_pair(iter_frame->second->id, ti0i1(1)));
						std::cout << "[INFO] ignoring : " << iter_frame->second->id << " distance threshold : " << ti0i1(1) << std::endl;
					}
					count_temp++;
				}
			}
			else
			{
				last_att_ave = att_ave;
			}
			//	Transformation between the reference camera frame(IPM frame) and the latest camera frame.
			Eigen::Matrix3d R_ilatest_iave = a2mat(Vector3d(att_diff(0) /*- att_ave(0) * 0.25*/, att_diff(1), 0.0));
			Eigen::MatrixXd R_clatest_cave = config.cam.Ric.transpose()*R_ilatest_iave*config.cam.Ric;

			R_c1c0 = (R_c1c0.transpose() * R_clatest_cave).transpose();
		}

		ipm_processer.updateCameraGroundGeometry(gv::CameraGroundGeometry(R_c1c0, config.cam.cg.getH()));

		// Load image.
		cv::Mat img_raw = cv::imread( raw_dir +"/" + image_iter->second);

		// IPM tranform.
		cv::Mat ipm = ipm_processer.genIPM(img_raw, true);

		// Load image(semantic).
		cv::Mat img_semantic;
		if (image_iter->second.find("/") == string::npos)
			img_semantic = cv::imread(semantic_dir + "/" + image_iter->second.substr(0, image_iter->second.find(".")) + ".png", -1);
		else
			img_semantic = cv::imread(semantic_dir + "/" + image_iter->second.substr(image_iter->second.find("/"), image_iter->second.find(".") - image_iter->second.find("/")) + ".png", -1);
		
		cv::Mat img_semantic_temp = cv::Mat(img_raw.rows, img_raw.cols, CV_8UC1);
		img_semantic_temp.setTo(cv::Scalar(0));
		img_semantic.copyTo(img_semantic_temp(cv::Rect(0, img_raw.rows - img_semantic.rows,
			img_raw.cols, img_semantic.rows)));
		img_semantic = img_semantic_temp;
		cv::Mat ipm_semantic = ipm_processer.genIPM(img_semantic, true);

		// Generate instance-level patches.
		auto this_frame = generateInstancePatch(config, ipm_processer, ipm, ipm_semantic);
		this_frame.R = this_pose->second.R;
		this_frame.t = this_pose->second.t;
		this_frame.time = this_pose->first;
		this_frame.generateMetricPatches(config, ipm_processer);
		road_map.addFrame(this_frame);
		if (frame_count % config.mapping_step == 1)
		{
			road_map.mergePatches(config, 0, this_frame.R, this_frame.t);
			//Sleep(1000);
		}

		// For visualization.
		cv::Mat ipm_color(ipm_semantic.rows, ipm_semantic.cols, CV_8UC3);
		ipm_color.setTo(cv::Vec3b(0, 0, 0));
		genColorLabel(ipm_semantic, ipm_color);
		cv::Mat ipm_semantic_cluster = cv::Mat(config.cam.IPM_HEIGHT, config.cam.IPM_WIDTH, CV_8UC3);
		ipm_color.copyTo(ipm_semantic_cluster);

		// OpenCV visualization
		if (config.enable_vis_image)
		{
			for (auto iter_class = this_frame.patches.begin(); iter_class != this_frame.patches.end(); iter_class++)
			{
				for (auto iter_instance = iter_class->second.begin(); iter_instance != iter_class->second.end(); iter_instance++)
				{
					cv::circle(ipm_semantic_cluster, cv::Point2f((*iter_instance)->mean(0), (*iter_instance)->mean(1)), 1, cv::Scalar(255, 255, 255));

					if ((iter_class->first == PatchType::SOLID || iter_class->first == PatchType::STOP) && (*iter_instance)->line_valid)
					{
						double direction_angle = atan2((*iter_instance)->direction(1), (*iter_instance)->direction(0)) + M_PI / 2; // std::cerr << direction_angle << std::endl;
						Matrix3d direction_rotation;
						direction_rotation << cos(direction_angle), sin(direction_angle), 0,
							-sin(direction_angle), cos(direction_angle), 0,
							0, 0, 1;

						for (int z = -(*iter_instance)->eigen_value[0] * 2;
							z < (*iter_instance)->eigen_value[0] * 2; z++)
						{
							double y0, y1, x0, x1;
							y0 = (*iter_instance)->mean(1) + z;
							y1 = (*iter_instance)->mean(1) + z + 1;
							x0 = 0;
							x1 = 0;
							for (int i = 0; i < (*iter_instance)->line_koef.rows(); i++)
							{
								x0 += pow(y0, i) * (*iter_instance)->line_koef(i);
								x1 += pow(y1, i) * (*iter_instance)->line_koef(i);
							}
							Vector3d pt0_img = direction_rotation.transpose()
								* (Vector3d(x0, y0, 1) - (*iter_instance)->mean) + (*iter_instance)->mean;
							Vector3d pt1_img = direction_rotation.transpose()
								* (Vector3d(x1, y1, 1) - (*iter_instance)->mean) + (*iter_instance)->mean;

							x0 = pt0_img.x();
							y0 = pt0_img.y();
							x1 = pt1_img.x();
							y1 = pt1_img.y();
							cv::line(ipm_semantic_cluster, cv::Point2f(x0, y0), cv::Point2f(x1, y1), cv::Scalar(0, 255, 0));
						}
						for (int i = 0; i < (*iter_instance)->line_points.size(); i++)
						{
							cv::circle(ipm_semantic_cluster, cv::Point2f((*iter_instance)->line_points[i].x(), (*iter_instance)->line_points[i].y()), 5, cv::Scalar(255, 255, 0));
						}
					}
				}
			}
			cv::imshow("img_raw", img_raw);
			cv::imshow("ipm", ipm);
			cv::Mat img_semantic_color = cv::Mat::zeros(img_raw.rows, img_raw.cols, CV_8UC3);
			genColorLabel(img_semantic, img_semantic_color);
			cv::imshow("img_semantic_color", img_semantic_color);
			cv::imshow("ipm_semantic_cluster", ipm_semantic_cluster);
			cv::waitKey(1);
		}

		// 3D visualization.
		if (config.enable_vis_3d)
		{
			vis_instances.clear();
			vis_points_this_frame.clear();
			if (vis_points_this_frame.size() == 0)
				for (auto iter_class = this_frame.patches.begin(); iter_class != this_frame.patches.end(); iter_class++)
					vis_points_this_frame.push_back(vector<Vector3d>());

			int ccount = 0;
			for (auto iter_class = this_frame.patches.begin(); iter_class != this_frame.patches.end(); iter_class++)
			{
				//vis_points_this_frame.push_back(vector<Vector3d>());
				for (auto iter_instance = iter_class->second.begin(); iter_instance != iter_class->second.end(); iter_instance++)
				{
					for (auto& p : (*iter_instance)->points_metric) vis_points_this_frame[ccount].push_back(this_frame.t + this_frame.R * p);
				}
				ccount++;
			}
			visualize_roadmap(road_map, vis_instances);
			old_poses.push_back(make_pair(this_frame.R, this_frame.t));


			visualize_vehicle(this_pose->second.t, m2att(this_pose->second.R), vis_instances);
			viewer.SetFrames(vector<pair<Matrix3d, Vector3d>>(1, make_pair(this_pose->second.R, this_pose->second.t)));
			viewer.SetInstances(vis_instances);
			viewer.SetPointCloudSemantic(vis_points_this_frame);
		}

		//viewer.ScreenShot(image_iter->second.substr(image_iter->second.find("/")+1, image_iter->second.find(".") - image_iter->second.find("/")-1) + ".png");
		//std::cerr << image_iter->second.substr(image_iter->second.find("/")+1, image_iter->second.find(".") - image_iter->second.find("/")-1) + ".png" << std::endl;
		
		
		all_frames.insert(make_pair(image_iter->first, make_shared<RoadInstancePatchFrame>(this_frame)));
		std::cout << "[INFO] Incremental mapping: (frame) " << frame_count << " " 
			<<"(time) "<< setprecision(3) << setiosflags(ios::fixed) << all_frames.rbegin()->first << std::endl;
		//cin.get();
	}

	return 0;
}



