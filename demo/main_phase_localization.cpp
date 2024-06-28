
#include <unordered_map>
#include "roadlib.h"
#include "gviewer.h"
#include "fgo.hpp"
#include <fstream>
#include "visualization.h"

int main_phase_localization(const SensorConfig& config,
	const map<double, string>& camstamp,
	const string& raw_dir, const string& semantic_dir,
	RoadInstancePatchMap& road_map,
	const Trajectory& traj_vio_user,
	const pair<double, Tpoint>& initial_guess,
	Trajectory& traj_user, string result_file)
{
	gv::IPMProcesser ipm_processer(config.cam, gv::IPMType::NORMAL);

	ofstream ofs_result(result_file);
	road_map.buildKDTree();

	std::cerr << "[ PHASE ] : Map-aided localization." << std::endl;

	// A pose window for smoothing.
	deque<pair<Eigen::Matrix3d, Eigen::Vector3d>> pose_history;

	Vector3d graph_pos_ref = road_map.ref;
	Matrix3d graph_Rne_ref = calcRne(road_map.ref);

	map<double, shared_ptr<RoadInstancePatchFrame>> all_frames;
	map<double, long long> keyframes_flag;
	long long keyframes_count=0;
	int frame_count = 0;
	Eigen::Vector3d last_att_ave;
	for (auto image_iter = camstamp.begin(); image_iter != camstamp.end(); image_iter++)
	{
		if (image_iter->first < config.t_start) continue;
		if (image_iter->first > config.t_end) break;

		frame_count++;

		Eigen::Matrix3d R_c1c0 = config.cam.cg.getR();

		// Get the latest IMU pose.
		auto this_pose = traj_vio_user.poses.lower_bound(image_iter->first - 0.001);
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

			if (fabs((att_ave(0) - last_att_ave(0)) * R2D) > config.large_slope_thresold / config.pose_smooth_window && pose_history.size() > 5) // big road slope change detected
			{
				std::cerr << "[INFO] Large slope detected!" << std::endl;
				pose_history.clear();
				pose_history.push_back(make_pair(this_pose->second.R, this_pose->second.t));
				last_att_ave = att_this;
				// int count_temp = 0;
				// for (auto iter_frame = all_frames.rbegin(); iter_frame != all_frames.rend() && count_temp < 20; iter_frame++)
				// {
				// 	Vector3d ti0i1 = iter_frame->second->R.transpose()*(this_pose->second.t - iter_frame->second->t);
				// 	// std::cout << "[INFO] ti0i1 : " << ti0i1.transpose() << std::endl;
				// 	if (ti0i1(1) < 13.5 && (ti0i1(0)) < 5)
				// 		for(auto iter_class = iter_frame->second->patches.begin();iter_class!=iter_frame->second->patches.end();iter_class++)
				// 			for(int ipatch = 0;ipatch<iter_class->second.size();ipatch++)
				// 			 iter_class->second[ipatch]->valid_add_to_map = false;
				// 	count_temp++;
				// }
			}
			else
			{
				last_att_ave = att_ave;
			}
			//	Transformation between the reference camera frame(IPM frame) and the latest camera frame.
			Eigen::Matrix3d R_ilatest_iave = a2mat(Vector3d(att_diff(0) /*- att_ave(0) * 0.25*/, att_diff(1), 0.0));
			Eigen::MatrixXd R_clatest_cave = config.cam.Ric.transpose() * R_ilatest_iave * config.cam.Ric;

			R_c1c0 = (R_c1c0.transpose() * R_clatest_cave).transpose();
		}

		ipm_processer.updateCameraGroundGeometry(gv::CameraGroundGeometry(R_c1c0, config.cam.cg.getH()));

		// Load image.
		cv::Mat img_raw = cv::imread(raw_dir + "/" + image_iter->second);

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
		auto this_frame_ptr = make_shared<RoadInstancePatchFrame>(generateInstancePatch(config, ipm_processer, ipm, ipm_semantic));
		auto& this_frame = *this_frame_ptr;
		this_frame.R = this_pose->second.R; // At this moment, this pose is still odometry pose.
		this_frame.t = this_pose->second.t;
		this_frame.time = this_pose->first;
		this_frame.generateMetricPatches(config, ipm_processer);

		
		int old_old_visualize_size;

		// Sliding window management.
		if(all_frames.size()>0)
		{
			auto iter_traj_vio_user = traj_vio_user.poses.lower_bound(all_frames.rbegin()->first - 0.001);
			auto iter_temp = all_frames.rbegin(); iter_temp++;
			auto iter_traj_vio_user_before = traj_vio_user.poses.lower_bound(iter_temp->first - 0.001);
			double dist_from_last_keyframe = (iter_traj_vio_user->second.t - iter_traj_vio_user_before->second.t).norm();

			if (dist_from_last_keyframe <config.localization_min_keyframe_dist && all_frames.size()>1)
			{
				// std::cout << "[INFO] Skip keyframe: " << setprecision(3) << setiosflags(ios::fixed) << all_frames.rbegin()->first
				// 	<< " Travelled distance : " << setprecision(3) << setiosflags(ios::fixed) << dist_from_last_keyframe << std::endl;
				all_frames.erase(all_frames.rbegin()->first);
			}
			else
			{
				keyframes_flag[all_frames.rbegin()->first] = keyframes_count++;
			}
		}

		all_frames.insert(make_pair(image_iter->first, this_frame_ptr));
		keyframes_flag[all_frames.rbegin()->first] = -1;

		while (all_frames.size() > config.localization_max_windowsize)
		{
			all_frames.erase(all_frames.begin());
		}

		// Predict current pose based on last pose estimation and relative pose.
		if ( all_frames.size() > 0)
		{
			auto iter_frame = all_frames.rbegin();
			auto iter_frame_before = all_frames.rbegin();
			iter_frame_before--;

			auto iter_traj_vio_user = traj_vio_user.poses.lower_bound(iter_frame->first - 0.001);
			auto iter_traj_vio_user_before = traj_vio_user.poses.lower_bound(iter_frame_before->first - 0.001);
			if (iter_traj_vio_user != traj_vio_user.poses.end() && fabs(iter_traj_vio_user->first - iter_frame->first) < 0.001
				&& iter_traj_vio_user_before != traj_vio_user.poses.end() && fabs(iter_traj_vio_user_before->first - iter_frame_before->first) < 0.001)
			{
				Vector3d t0 = iter_traj_vio_user_before->second.t;
				Matrix3d R0 = iter_traj_vio_user_before->second.R;
				Vector3d t1 = iter_traj_vio_user->second.t;
				Matrix3d R1 = iter_traj_vio_user->second.R;

				Matrix3d R01 = R0.transpose() * R1;
				Quaterniond q01 = Quaterniond(R01).normalized();
				Vector3d t01 = R0.transpose() * (t1 - t0);

				// Current pose is updated.
				(*iter_frame->second).t = (*iter_frame_before->second).t + (*iter_frame_before->second).R * t01;
				(*iter_frame->second).R = (*iter_frame_before->second).R * R01;
			}
		}

		// For visualization.
		if (config.enable_vis_image)
		{
			cv::Mat ipm_color(ipm_semantic.rows, ipm_semantic.cols, CV_8UC3);
			ipm_color.setTo(cv::Vec3b(0, 0, 0));
			genColorLabel(ipm_semantic, ipm_color);
			cv::Mat ipm_semantic_cluster = cv::Mat(config.cam.IPM_HEIGHT, config.cam.IPM_WIDTH, CV_8UC3);
			ipm_color.copyTo(ipm_semantic_cluster);

			cv::imshow("ipm", ipm);
			cv::imshow("ipm_semantic_cluster", ipm_semantic_cluster);
			cv::waitKey(1);
		}

		vector<vector<Vector3d>> vis_points_this_frame;
		vector<VisualizedInstance> vis_instances;

		if (config.enable_vis_3d)
		{
			visualize_roadmap(road_map, vis_instances);
			viewer.SetInstances(vis_instances);

			// Pre-iteration visualization.
			for (auto iter_class = this_frame.patches.begin(); iter_class != this_frame.patches.end(); iter_class++)
			{
				vis_points_this_frame.push_back(vector<Vector3d>());
				for (auto iter_instance = iter_class->second.begin(); iter_instance != iter_class->second.end(); iter_instance++)
				{
					for (auto& p : (*iter_instance)->points_metric) vis_points_this_frame.back().push_back(this_frame.t + this_frame.R * p);
				}
			}
			viewer.SetPointCloudSemantic(vis_points_this_frame);
			viewer.SetFrames(vector<pair<Matrix3d, Vector3d>>(1, make_pair(this_frame.R, this_frame.t)));
		}

		// Start iteration.
		for (int i_iter = 0; i_iter < 3; i_iter++)
		{
			std::vector<ceres::ResidualBlockId> psrIDs;
			ceres::Problem problem;
			ceres::Solver::Options options;
			ceres::Manifold* local_parameterization = new ceres::QuaternionManifold();
			vector<t_graphpose> all_graph_poses;
			map<pair<long long, int>, t_patch_est> map_patch_est; // All the patches in the map that need to be updated.

			// Clear temporary visualization.
			if (i_iter > 0)
			{
				vis_instances.resize(old_old_visualize_size);
			}
			old_old_visualize_size = vis_instances.size();

			int ccount = 0;
			
			// Nearest matching.
			for (auto iter_frame = all_frames.begin(); iter_frame != all_frames.end(); iter_frame++, ccount++)
			{
				bool enable_match = (keyframes_flag[iter_frame->first] % config.localization_every_n_frames ==0 || ccount > all_frames.size() - config.localization_force_last_n_frames);
				map<PatchType, vector<pair<int, int>>> class_pairs;

				map<PatchType, map<int, pair<int, vector<pair<int, int>>>>> line_pairs;

				auto& this_graph_frame = (*iter_frame->second);
				if (enable_match)
				{
					if (i_iter > 0)
						class_pairs = road_map.mapMatch(config, this_graph_frame, 1);
					else
						class_pairs = road_map.mapMatch(config, this_graph_frame, 1);

					vector<PatchType> line_class_list = { PatchType::SOLID,PatchType::STOP };
					for (auto line_class : line_class_list)
					{
						line_pairs[line_class] = map<int, pair<int, vector<pair<int, int>>>>();
						for (auto iter_line_match = class_pairs[line_class].begin(); iter_line_match != class_pairs[line_class].end(); iter_line_match++)
						{
							int frame_line_count = iter_line_match->second;
							int map_line_count = iter_line_match->first;
							if (i_iter > 0)
								line_pairs[line_class].emplace(frame_line_count, make_pair(map_line_count, road_map.getLineMatch(config, this_graph_frame, line_class,
									frame_line_count, map_line_count, 1)));
							else
								line_pairs[line_class].emplace(frame_line_count, make_pair(map_line_count, road_map.getLineMatch(config, this_graph_frame, line_class,
									frame_line_count, map_line_count, 1)));
						}

#ifdef PRINT_INFO
						for (int i = 0; i < line_pairs[frame_line_count].second.size(); i++)
							std::cerr << line_pairs[frame_line_count].second[i].first << " " << line_pairs[frame_line_count].second[i].second << std::endl;
#endif
					}
				}
				if (class_pairs[PatchType::DASHED].size() == 0 && class_pairs[PatchType::GUIDE].size() == 0
					&& class_pairs[PatchType::STOP].size() == 0 && class_pairs[PatchType::SOLID].size() <= 2)
				{
					class_pairs.clear();
					line_pairs.clear();
				}

				// Add pose parameters (ceres).
				Quaterniond this_graph_frame_q = Quaterniond(this_graph_frame.R).normalized();

				double* p3 = new double[3];
				p3[0] = this_graph_frame.t(0);
				p3[1] = this_graph_frame.t(1);
				p3[2] = this_graph_frame.t(2);

				double* p4 = new double[4];
				p4[0] = this_graph_frame_q.w();
				p4[1] = this_graph_frame_q.x();
				p4[2] = this_graph_frame_q.y();
				p4[3] = this_graph_frame_q.z();

				all_graph_poses.push_back({ iter_frame->first ,this_graph_frame.R,this_graph_frame.t,p3,p4 });
				problem.AddParameterBlock(all_graph_poses.back().q_array, 4, local_parameterization);
				problem.AddParameterBlock(all_graph_poses.back().t_array, 3);
		
				// Add landmark parameters (ceres).
				// Attention!!!: these parameters are set constant.
				for (auto iter_class = class_pairs.begin(); iter_class != class_pairs.end(); iter_class++)
				{
					if (iter_class->first == PatchType::DASHED || iter_class->first == PatchType::GUIDE)
					{
						for (int j = 0; j < iter_class->second.size(); j++)
						{
							auto& this_patch_map = road_map.patches[iter_class->first][iter_class->second[j].first];
							if (map_patch_est.find(make_pair(this_patch_map->id, 0)) != map_patch_est.end()) continue;

							// For patch-like instances, add the four bounding points to parameters.
							for (int iii = 0; iii < 4; iii++)
							{
								map_patch_est[make_pair(this_patch_map->id, iii)] = t_patch_est(this_patch_map->b_point_metric[iii]);
								problem.AddParameterBlock(map_patch_est[make_pair(this_patch_map->id, iii)].p, 3);
								problem.SetParameterBlockConstant(map_patch_est[make_pair(this_patch_map->id, iii)].p);
							}
#ifdef PRINT_INFO
							std::cerr << "Add Dot patch : " << this_patch_map->id << " " << 0 << std::endl;
#endif
						}
					}
					else if (iter_class->first == PatchType::SOLID || iter_class->first == PatchType::STOP)
					{
						for (int j = 0; j < iter_class->second.size(); j++)
						{
							auto& this_patch_map = road_map.patches[iter_class->first][iter_class->second[j].first];

							for (int k = 0; k < line_pairs[iter_class->first][iter_class->second[j].second].second.size(); k++)
							{
								int instance_vertex_map = line_pairs[iter_class->first][iter_class->second[j].second].second[k].second;
								for (int zzz = 0; zzz < 2; zzz++)
									if (map_patch_est.find(make_pair(this_patch_map->id, instance_vertex_map + zzz)) == map_patch_est.end())
									{
										map_patch_est[make_pair(this_patch_map->id, instance_vertex_map + zzz)] = t_patch_est(this_patch_map->line_points_metric[instance_vertex_map + zzz]);
										problem.AddParameterBlock(map_patch_est[make_pair(this_patch_map->id, instance_vertex_map + zzz)].p, 3);
										problem.SetParameterBlockConstant(map_patch_est[make_pair(this_patch_map->id, instance_vertex_map + zzz)].p);
#ifdef PRINT_INFO
										std::cerr << "Add Line patch : " << this_patch_map->id << " " << instance_vertex_map + zzz << std::endl;
#endif
									}
							}
						}
					}
				}

				// Initial guess factor (ceres).
				if (fabs(iter_frame->first - initial_guess.first) < 0.001)
				{
					ceres::LossFunction* gnss_loss_function = new ceres::TrivialLoss();
					Vector3d t0 = initial_guess.second.t;
					Matrix3d R0 = initial_guess.second.R;
					Quaterniond q0 = Quaterniond(R0).normalized();
					ceres::CostFunction* gnss_pose_factor;
					gnss_pose_factor = PoseVarError::Create(t0.x(), t0.y(), t0.z(),
						q0.w(), q0.x(), q0.y(), q0.z(), 10, 10, 10, 0.01, 0.01, 0.04);
					auto ID = problem.AddResidualBlock(gnss_pose_factor, gnss_loss_function, all_graph_poses.back().q_array,
						all_graph_poses.back().t_array);
					psrIDs.push_back(ID);
				}

				// Relative pose factor (ceres).
				if (iter_frame != all_frames.begin())
				{
					auto iter_frame_before = iter_frame;
					iter_frame_before--;

					auto iter_traj_vio_user = traj_vio_user.poses.lower_bound(iter_frame->first - 0.001);
					auto iter_traj_vio_user_before = traj_vio_user.poses.lower_bound(iter_frame_before->first - 0.001);
					if (iter_traj_vio_user != traj_vio_user.poses.end() && fabs(iter_traj_vio_user->first - iter_frame->first) < 0.001
						&& iter_traj_vio_user_before != traj_vio_user.poses.end() && fabs(iter_traj_vio_user_before->first - iter_frame_before->first) < 0.001)
					{
						ceres::LossFunction* vio_loss_function = new ceres::TrivialLoss();
						// ceres::LossFunction* vio_loss_function = new ceres::HuberLoss(1.0);

						Vector3d t0 = iter_traj_vio_user_before->second.t;
						Matrix3d R0 = iter_traj_vio_user_before->second.R;
						Vector3d t1 = iter_traj_vio_user->second.t;
						Matrix3d R1 = iter_traj_vio_user->second.R;

						Matrix3d R01 = R0.transpose() * R1;
						Quaterniond q01 = Quaterniond(R01).normalized();
						Vector3d t01 = R0.transpose() * (t1 - t0);

						double dist = t01.norm();
						dist = dist < 0.1 ? 0.1 : dist;
						ceres::CostFunction* relative_pose_factor = RelativeRTVarError::Create(t01.x(), t01.y(), t01.z(),
							q01.w(), q01.x(), q01.y(), q01.z(),
							0.05 * dist, 0.1 * dist, 0.05 * dist,
							0.01 , 0.01, 0.01);

						problem.AddResidualBlock(relative_pose_factor, vio_loss_function, (all_graph_poses.rbegin() + 1)->q_array,
							(all_graph_poses.rbegin() + 1)->t_array,
							all_graph_poses.rbegin()->q_array,
							all_graph_poses.rbegin()->t_array);
					}
				}

				// Map matching (point-to-line) factor (ceres).
				ceres::LossFunction* map_loss_function = new ceres::HuberLoss(1.0);
				VisualizedInstance vis_instance;
				vis_instance.type = VisualizedPatchType::LINE_SEGMENT;
				vis_instance.pts = vector<Vector3d>(3);
				vis_instance.pts_color = vector<Vector3d>(3, Eigen::Vector3d(1.0, 0, 1.0));
				vis_instance.alpha = 1.0f;
				vis_instance.linewidth = 1.0f;

				for (auto iter_class = class_pairs.begin(); iter_class != class_pairs.end(); iter_class++)
				{
					for (int j = 0; j < iter_class->second.size(); j++)
					{
						auto& this_patch_map = road_map.patches[iter_class->first][iter_class->second[j].first];
						auto& this_patch_frame = this_graph_frame.patches[iter_class->first][iter_class->second[j].second];

						if (iter_class->first == PatchType::DASHED || iter_class->first == PatchType::GUIDE)
						{
							for (int k = 0; k < 4; k++)
							{
								Eigen::Vector3d pos_meas = (this_patch_frame->b_point_metric[k % 4] + this_patch_frame->b_point_metric[(k + 1) % 4])/2;
								ceres::CostFunction* map_factor = MapLineToFramePointFactor::Create(pos_meas.x(), pos_meas.y(), pos_meas.z(),
									this_patch_frame->b_unc_dist[k],
									this_patch_frame->b_unc_dist[k],
									this_patch_frame->b_unc_dist[k]);
								auto ID = problem.AddResidualBlock(map_factor, map_loss_function, all_graph_poses.back().q_array,
									all_graph_poses.back().t_array,
									map_patch_est[make_pair(this_patch_map->id, k % 4)].p,
									map_patch_est[make_pair(this_patch_map->id, (k + 1) % 4)].p);
								psrIDs.push_back(ID);
								pos_meas = this_graph_frame.R * pos_meas + this_graph_frame.t;
								vis_instance.pts[0] = Vector3d(map_patch_est[make_pair(this_patch_map->id, k % 4)].p);
								vis_instance.pts[1] = pos_meas;
								vis_instance.pts[2] = Vector3d(map_patch_est[make_pair(this_patch_map->id, (k+1) % 4)].p);
								vis_instances.push_back(vis_instance);
							}
						}
						else if (iter_class->first == PatchType::SOLID || iter_class->first == PatchType::STOP)
						{
							for (int k = 0; k < line_pairs[iter_class->first][iter_class->second[j].second].second.size(); k++)
							{
								int instance_vertex_frame = line_pairs[iter_class->first][iter_class->second[j].second].second[k].first;
								int instance_vertex_map = line_pairs[iter_class->first][iter_class->second[j].second].second[k].second;

								Vector3d pos_meas = this_patch_frame->line_points_metric[instance_vertex_frame];
								double uncertainty = sqrt(this_patch_frame->line_points_uncertainty[instance_vertex_frame](0, 0) +
									                      this_patch_frame->line_points_uncertainty[instance_vertex_frame](1, 1) +
									                      this_patch_frame->line_points_uncertainty[instance_vertex_frame](2, 2))/sqrt(3.0);
								ceres::CostFunction* map_factor = MapLineToFramePointFactor::Create(pos_meas.x(), pos_meas.y(), pos_meas.z(), 
									uncertainty,
									uncertainty,
									uncertainty);
								auto ID = problem.AddResidualBlock(map_factor, map_loss_function, all_graph_poses.back().q_array,
									all_graph_poses.back().t_array,
									map_patch_est[make_pair(this_patch_map->id, instance_vertex_map)].p,
									map_patch_est[make_pair(this_patch_map->id, instance_vertex_map + 1)].p);
								psrIDs.push_back(ID);
								pos_meas = this_graph_frame.R * pos_meas + this_graph_frame.t;
								vis_instance.pts[0] = Vector3d(map_patch_est[make_pair(this_patch_map->id, instance_vertex_map)].p);
								vis_instance.pts[1] = pos_meas;
								vis_instance.pts[2] = Vector3d(map_patch_est[make_pair(this_patch_map->id, instance_vertex_map + 1)].p);
								vis_instances.push_back(vis_instance);
							}
						}

					}
				}
			}


			// Per-iteration visualization.
			if (config.enable_vis_3d)
			{
				ccount = 0;
				for (auto iter_frame = all_frames.begin(); iter_frame != all_frames.end(); iter_frame++, ccount++)
				{
					if (fabs(iter_frame->first - (all_frames.rbegin()->first)) < 0.001)
					{
						VisualizedInstance vis_instance_vehicle;
						vis_instance_vehicle.type = VisualizedPatchType::BOX;
						vis_instance_vehicle.color[0] = 0.0;
						vis_instance_vehicle.color[1] = 0.0;
						vis_instance_vehicle.color[2] = 0.0;
						vis_instance_vehicle.t = iter_frame->second->t;
						vis_instance_vehicle.R = a2mat(m2att(iter_frame->second->R));
						vis_instance_vehicle.h = 1.753;
						vis_instance_vehicle.l = 1.849;
						vis_instance_vehicle.w = 4.690;
						vis_instances.push_back(vis_instance_vehicle);
					}
					else if (keyframes_flag[iter_frame->first] % config.localization_every_n_frames == 0 || (ccount > all_frames.size() - config.localization_force_last_n_frames))
					{
						VisualizedInstance vis_instance_vehicle;
						vis_instance_vehicle.type = VisualizedPatchType::BOX;
						vis_instance_vehicle.color[0] = 0.8;
						vis_instance_vehicle.color[1] = 0.8;
						vis_instance_vehicle.color[2] = 0.8;
						vis_instance_vehicle.t = iter_frame->second->t;
						vis_instance_vehicle.R = a2mat(m2att(iter_frame->second->R));
						vis_instance_vehicle.h = 1.753;
						vis_instance_vehicle.l = 1.849;
						vis_instance_vehicle.w = 4.690;
						vis_instances.push_back(vis_instance_vehicle);
					}
				}
				viewer.SetInstances(vis_instances);
			}

			options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);

			// Update poses.
			int frame_i = 0;
			for (auto iter_frame = all_frames.begin(); iter_frame != all_frames.end(); iter_frame++, frame_i++)
			{
				iter_frame->second->t = Vector3d(all_graph_poses[frame_i].t_array[0],
					all_graph_poses[frame_i].t_array[1],
					all_graph_poses[frame_i].t_array[2]);
				iter_frame->second->R = Quaterniond(all_graph_poses[frame_i].q_array[0],
					all_graph_poses[frame_i].q_array[1],
					all_graph_poses[frame_i].q_array[2],
					all_graph_poses[frame_i].q_array[3]).toRotationMatrix();
				delete[] all_graph_poses[frame_i].t_array;
				delete[] all_graph_poses[frame_i].q_array;
			}
		}

		// Post-optimization visualization.
		if (config.enable_vis_3d)
		{
			int ccount = 0;
			for (auto iter_frame = all_frames.begin(); iter_frame != all_frames.end(); iter_frame++, ccount++)
			{
				if (fabs(iter_frame->first - (all_frames.rbegin()->first)) < 0.001)
				{
					VisualizedInstance vis_instance_vehicle;
					vis_instance_vehicle.type = VisualizedPatchType::BOX;
					vis_instance_vehicle.color[0] = 0.0;
					vis_instance_vehicle.color[1] = 0.0;
					vis_instance_vehicle.color[2] = 0.0;
					vis_instance_vehicle.t = iter_frame->second->t;
					vis_instance_vehicle.R = a2mat(m2att(iter_frame->second->R));
					vis_instance_vehicle.h = 1.753;
					vis_instance_vehicle.l = 1.849;
					vis_instance_vehicle.w = 4.690;
					vis_instances.push_back(vis_instance_vehicle);
				}
				else if (keyframes_flag[iter_frame->first] % config.localization_every_n_frames == 0 || (ccount > all_frames.size() - config.localization_force_last_n_frames))
				{
					VisualizedInstance vis_instance_vehicle;
					vis_instance_vehicle.type = VisualizedPatchType::BOX;
					vis_instance_vehicle.color[0] = 0.8;
					vis_instance_vehicle.color[1] = 0.8;
					vis_instance_vehicle.color[2] = 0.8;
					vis_instance_vehicle.t = iter_frame->second->t;
					vis_instance_vehicle.R = a2mat(m2att(iter_frame->second->R));
					vis_instance_vehicle.h = 1.753;
					vis_instance_vehicle.l = 1.849;
					vis_instance_vehicle.w = 4.690;
					vis_instances.push_back(vis_instance_vehicle);
				}

			}

			viewer.SetInstances(vis_instances);
			vis_points_this_frame.clear();
			for (auto iter_class = this_frame.patches.begin(); iter_class != this_frame.patches.end(); iter_class++)
			{
				vis_points_this_frame.push_back(vector<Vector3d>());
				for (auto iter_instance = iter_class->second.begin(); iter_instance != iter_class->second.end(); iter_instance++)
				{
					for (auto& p : (*iter_instance)->points_metric) vis_points_this_frame.back().push_back(this_frame.t + this_frame.R * p);
				}
			}
			viewer.SetPointCloudSemantic(vis_points_this_frame);
		}

		// Output results.
		if (ofs_result.good())
		{
			Eigen::Vector3d pos = (graph_pos_ref + graph_Rne_ref.transpose() * all_frames.rbegin()->second->t);
			Eigen::Quaterniond qnb  = Eigen::Quaterniond(calcRne(pos) * graph_Rne_ref.transpose() * all_frames.rbegin()->second->R);

			ofs_result << setprecision(3) << setiosflags(ios::fixed) << all_frames.rbegin()->second->time << " "
				<< setprecision(3) << setiosflags(ios::fixed) << pos(0) << " "
				<< setprecision(3) << setiosflags(ios::fixed) << pos(1) << " "
				<< setprecision(3) << setiosflags(ios::fixed) << pos(2) << " "
				<< setprecision(10) << setiosflags(ios::fixed) << qnb.w() << " "
				<< setprecision(10) << setiosflags(ios::fixed) << qnb.x() << " "
				<< setprecision(10) << setiosflags(ios::fixed) << qnb.y() << " "
				<< setprecision(10) << setiosflags(ios::fixed) << qnb.z() << std::endl;
		}

		std::cout << "[INFO] Map-aided localization: (frame) " << frame_count << " " 
			<<"(time) "<< setprecision(3) << setiosflags(ios::fixed) << all_frames.rbegin()->first << std::endl;

		//viewer.ScreenShot(image_iter->second.substr(image_iter->second.find("/")+1, image_iter->second.find(".") - image_iter->second.find("/")-1) + ".png");

	}

	return 0;
}