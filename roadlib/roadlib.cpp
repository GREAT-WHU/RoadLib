/*******************************************************
 * Copyright (C) 2024, GREAT Group, Wuhan University
 * 
 * This file is part of RoadLib.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Yuxuan Zhou (yuxuanzhou@whu.edu.cn)
 *******************************************************/

#include "roadlib.h"

long long RoadInstancePatch::next_id = 0;
long long RoadInstancePatchFrame::next_id = 0;

vector<RoadInstancePatch> patches_temp;

int LabelClustering(const cv::Mat& ipm_semantic, cv::Mat& ipm_label, cv::Mat& stats, cv::Mat& centroids)
{
	cv::Mat ipm_label_stop, stats_stop, centroids_stop;
	cv::Mat ipm_semantic_temp;
	ipm_semantic.copyTo(ipm_semantic_temp);
	cv::Mat ipm_semantic_stop = cv::Mat::zeros(ipm_semantic.size(), CV_8UC1);
	ipm_semantic_stop.setTo(255, ipm_semantic == 5);
	ipm_semantic_temp.setTo(0, ipm_semantic == 5);

	cv::connectedComponentsWithStats(ipm_semantic_temp, ipm_label, stats, centroids, 4, CV_16U);
	cv::connectedComponentsWithStats(ipm_semantic_stop, ipm_label_stop, stats_stop, centroids_stop, 4, CV_16U);
	cv::Mat stats_all, centroids_all;
	if (stats_stop.rows > 1)
	{
		cv::vconcat(stats, stats_stop(cv::Rect(0, 1, stats_stop.cols, stats_stop.rows - 1)), stats);
		cv::vconcat(centroids, centroids_stop(cv::Rect(0, 1, centroids_stop.cols, centroids_stop.rows - 1)), centroids);
	}
	for (int i = 1; i < stats_stop.rows; i++)
	{
		ipm_label.setTo(stats.rows - stats_stop.rows + i, ipm_label_stop == i);
	}
	return 0;
}

Matrix3d calcUncertainty(const gv::IPMProcesser& ipm, double uI, double vI)
{

	double fx = ipm._fx;
	double fy = ipm._fy;

	double reso_temp = ipm._config.IPM_RESO;

	double d = ipm._config.cg.getH();

	Eigen::Matrix3d K_IPM_i_d;
	K_IPM_i_d << reso_temp, 0.0, -ipm._config.IPM_WIDTH * reso_temp / 2,
		0.0, reso_temp, -(ipm._config.IPM_HEIGHT - 1) * reso_temp,
		0.0, 0.0, d;

	Eigen::Matrix3d K_IPM = K_IPM_i_d.inverse() * d;

	Vector3d xyI, xy;
	double xI, yI, x, y;
	Eigen::MatrixXd F, F1, F2, F3;
	xyI = K_IPM.inverse() * Vector3d(uI, vI, 1);
	xI = xyI(0); yI = xyI(1);
	y = -1 / yI; x = xI * y;

	// Pixel error.
	F1 = (Eigen::MatrixXd(2, 2) << d / reso_temp, 0, 0, d / reso_temp).finished();
	F2 = (Eigen::MatrixXd(2, 2) << 1 / y, -x / y / y, 0, 1 / y / y).finished();
	F3 = (Eigen::MatrixXd(2, 2) << 1 / fx, 0, 0, 1 / fy).finished();
	F = F1 * F2 * F3;
	MatrixXd var_pixel = (Eigen::MatrixXd(2, 2) << 2, 0, 0, 2).finished();
	var_pixel = var_pixel * var_pixel;
	Matrix2d var_pixel_error = F * var_pixel * F.transpose(); // (in pixels)

	// Pitch error.
	F1 = (Eigen::MatrixXd(2, 2) << d / reso_temp, 0, 0, d / reso_temp).finished();
	F2 = (Eigen::MatrixXd(2, 1) << x / y / y, (1 + y * y) / (y * y)).finished();
	F = F1 * F2;
	double var_pitch = 0.3 / 57.3;
	var_pitch = var_pitch * var_pitch;
	Matrix2d var_pitch_error = F * var_pitch * F.transpose(); // (in pixels)

	// Height error.
	F = (Eigen::MatrixXd(2, 1) << xI / reso_temp, yI / reso_temp).finished();
	double var_D = 0.05;
	var_D = var_D * var_D;
	Matrix2d var_D_error = F * var_D * F.transpose(); // (in pixels)

	Matrix3d uncertainty = Matrix3d::Identity(3, 3);
	uncertainty.block(0, 0, 2, 2) = (var_pixel_error + var_pitch_error + var_D_error) * (reso_temp * reso_temp);
	uncertainty(2, 2) = pow((0.5 * sqrt(uncertainty(1, 1))), 2);
	return uncertainty;
}

RoadInstancePatchFrame generateInstancePatch(const SensorConfig& config, const gv::IPMProcesser& ipm, const cv::Mat& ipm_raw, const cv::Mat& ipm_class)
{
	cv::Mat ipm_instance, stats, centroids;
	LabelClustering(ipm_class, ipm_instance, stats, centroids);

	// Initialize the frame.
	RoadInstancePatchFrame frame;
	frame.patches[PatchType::SOLID] = vector<shared_ptr<RoadInstancePatch>>();
	frame.patches[PatchType::DASHED] = vector<shared_ptr<RoadInstancePatch>>();
	frame.patches[PatchType::GUIDE] = vector<shared_ptr<RoadInstancePatch>>();
	frame.patches[PatchType::ZEBRA] = vector<shared_ptr<RoadInstancePatch>>();
	frame.patches[PatchType::STOP] = vector<shared_ptr<RoadInstancePatch>>();

	patches_temp = vector<RoadInstancePatch>(centroids.rows);
	vector<int> patches_count(centroids.rows);

	for (int i = 0; i < centroids.rows; i++)
	{
		patches_temp[i].points.resize(stats.at<int>(i, cv::CC_STAT_AREA));
		patches_temp[i].top = stats.at<int>(i, cv::CC_STAT_TOP);
		patches_temp[i].left = stats.at<int>(i, cv::CC_STAT_LEFT);
		patches_temp[i].width = stats.at<int>(i, cv::CC_STAT_WIDTH);
		patches_temp[i].height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
	}

	// Count every patch.
	uint16_t cur_label;
	for (int i = 0; i < ipm_instance.rows; i++)
	{
		for (int j = 0; j < ipm_instance.cols; j++)
		{
			cur_label = ipm_instance.at<uint16_t>(i, j);
			patches_temp[cur_label].points[patches_count[cur_label]] = Eigen::Vector3d(j, i, 1);
			patches_temp[cur_label].road_class_count[gray2class(ipm_class.at<uchar>(i, j))]++;
			if (cur_label > 0)
			{
				if (i + 1 >= ipm_instance.rows || j <= 0 || j >= ipm_instance.cols - 1 ||
					ipm_raw.at<cv::Vec3b>(i + 1, j) == cv::Vec3b(0, 0, 0) ||
					ipm_raw.at<cv::Vec3b>(i + 1, j - 1) == cv::Vec3b(0, 0, 0) ||
					ipm_raw.at<cv::Vec3b>(i + 1, j + 1) == cv::Vec3b(0, 0, 0))
				{
					patches_temp[cur_label].out_range = true;
				}
			}
			patches_count[cur_label]++;
		}
	}
	for (int i = 0; i < patches_temp.size(); i++)
	{
		assert(patches_temp[i].points.size() == patches_count[i]);
	}

	// Clean redundant patches.
	auto it = patches_temp.begin();
	while (it != patches_temp.end()) {
		int max_class_count = 0;
		PatchType max_class = PatchType::EMPTY;
		for (auto class_it : it->road_class_count)
		{
			if (class_it.second > max_class_count)
			{
				max_class_count = class_it.second;
				max_class = class_it.first;
			}
		}
		it->road_class = max_class;
		if (max_class == PatchType::EMPTY)
		{
			it = patches_temp.erase(it);
			continue;
		}

		bool out_range = false;
		if (it->points.size() < config.patch_min_size
			|| (it->road_class == PatchType::DASHED && out_range)
			|| (it->road_class == PatchType::GUIDE && out_range)
			)
		{
			it = patches_temp.erase(it);
			continue;
		}
		it++;
	}

	// Calculate patch characteristics.
	for (int i = 0; i < patches_temp.size(); i++)
	{
		auto& patch = patches_temp[i];
		patch.mean.setZero();
		patch.cov.setZero();
		Eigen::Vector3d vector_temp;

		for (int j = 0; j < patch.points.size(); j++)
		{
			patch.mean += patch.points[j];
		}
		patch.mean /= patch.points.size();

		for (int j = 0; j < patch.points.size(); j++)
		{
			vector_temp = patch.points[j] - patch.mean;
			patch.cov += vector_temp * vector_temp.transpose();
		}
		patch.cov /= patch.points.size();

		JacobiSVD<Eigen::MatrixXd> svd(patch.cov, ComputeThinU | ComputeThinV);
		Matrix3d V = svd.matrixV(), U = svd.matrixU();
		Matrix3d S = U.inverse() * patch.cov * V.transpose().inverse();
		patch.eigen_value[0] = sqrt(S(0, 0));
		patch.eigen_value[1] = sqrt(S(1, 1));
		patch.eigen_value[2] = sqrt(S(2, 2));

		patch.direction = U.block(0, 0, 3, 1);
		patch.mean_uncertainty = calcUncertainty(ipm, patch.mean(0), patch.mean(1));


		// >> temporarily!!!!!
		if (patch.direction(1) < 0) patch.direction *= -1;

		if (patch.eigen_value[0] / patch.eigen_value[1] > 2
			&& (patches_temp[i].road_class == PatchType::SOLID || patches_temp[i].road_class == PatchType::STOP))
		{
			auto points_direction = patch.points;
			double direction_angle = atan2(patch.direction(1), patch.direction(0)) + M_PI / 2; // std::cerr << direction_angle << std::endl;
			Matrix3d direction_rotation;
			direction_rotation << cos(direction_angle), sin(direction_angle), 0,
				-sin(direction_angle), cos(direction_angle), 0,
				0, 0, 1;
			for (int i = 0; i < points_direction.size(); i++)
			{
				points_direction[i] = direction_rotation * (patch.points[i] - patch.mean) + patch.mean;
			}

			// Line-like point clouds to skeleton points.
			if (PointCloud2Curve2D(points_direction, 4, patch.line_koef) < 0)
				patch.line_valid = false;
			else
			{
				patch.line_valid = true;
				double yy, xx;
				Vector3d pt_img;
				double min_uncertainty = 1e9;
				double reso_temp = ipm._config.IPM_RESO;
				for (int ll = (int)ceil(-patch.eigen_value[0] * 2 / (0.5 / reso_temp));
					ll < patch.eigen_value[0] * 2 / (0.5 / reso_temp); ll++)
				{
					yy = patch.mean(1) + ll * 0.5 / reso_temp;
					xx = 0.0;
					for (int i = 0; i < patch.line_koef.rows(); i++)
						xx += pow(yy, i) * patch.line_koef(i);
					pt_img = direction_rotation.transpose() * (Vector3d(xx, yy, 1) - patch.mean) + patch.mean;

					if (pt_img(1) > (patch.top + patch.height)) continue;

					double valid_distance = 20;
					if (patches_temp[i].road_class == PatchType::SOLID) valid_distance = config.patch_solid_max_dist;
					else if (patches_temp[i].road_class == PatchType::STOP) valid_distance = config.patch_stop_max_dist;

					if ((ipm._config.IPM_HEIGHT - pt_img(1)) * reso_temp < valid_distance)
					{
						patch.line_points.push_back(pt_img);
						patch.line_points_uncertainty.push_back(calcUncertainty(ipm, pt_img(0), pt_img(1)));
						min_uncertainty = min(min_uncertainty,
							sqrt(patch.line_points_uncertainty.back()(0, 0) + patch.line_points_uncertainty.back()(1, 1)));
					}
				}
				if (patch.line_points.size() < 2 || min_uncertainty > 1.0)
					patch.line_valid = false;
			}
		}
		else
			patch.line_valid = false;
	}

	// We collect nearby lane line segments to determine the direction of patch-like markings.
	// This could be improved.
	vector<pair<Vector3d, Vector3d>> direction_field;
	for (int i = 0; i < patches_temp.size(); i++)
	{
		auto& patch = patches_temp[i];

		if (patch.road_class == PatchType::DASHED)
			direction_field.push_back(make_pair(patch.mean, patch.direction));
		if (patches_temp[i].road_class == PatchType::SOLID && patch.line_valid)
			for (int ii = 0; ii < patch.line_points.size() - 1; ii++)
				direction_field.push_back(make_pair(patch.line_points[ii], patch.line_points[ii + 1] - patch.line_points[ii]));
	}

	//** Compute bounding boxes.
	// Use the local direction field to determine the direction of a patch-like marker.
	for (int i = 0; i < patches_temp.size(); i++)
	{
		auto patch = make_shared<RoadInstancePatch>(patches_temp[i]);

		if (patch->road_class == PatchType::GUIDE || patch->road_class == PatchType::DASHED)
		{
			double min_dist = 999999;
			int min_ii = -1;
			for (int ii = 0; ii < direction_field.size(); ii++)
			{
				double dist = (direction_field[ii].first - patch->mean).norm();
				if (dist < min_dist)
				{
					min_dist = dist;
					min_ii = ii;
				}
			}
			if (min_dist < 10000)
			{
				Eigen::Vector3d b_direction = direction_field[min_ii].second / direction_field[min_ii].second.norm();
				if (b_direction(1) < 0)  b_direction *= -1;

				double bb= -1e9; Vector3d tt_support;
				double tt = 1e9; Vector3d bb_support;
				double ll = 1e9; Vector3d ll_support;
				double rr = -1e9; Vector3d rr_support;

				double direction_angle = -atan2(b_direction(1), b_direction(0)) + M_PI / 2;
				Matrix3d direction_rotation;
				direction_rotation << cos(direction_angle), sin(direction_angle), 0,
					-sin(direction_angle), cos(direction_angle), 0,
					0, 0, 1;

				for (int jj = 0; jj < patch->points.size(); jj++)
				{
					Vector3d pt = direction_rotation.transpose() * (patch->points[jj] - patch->mean);
					if (pt.x() < ll) { ll = pt.x(); ll_support = patch->points[jj]; }
					if (pt.x() > rr) { rr = pt.x(); rr_support = patch->points[jj]; }
					if (pt.y() < tt) { tt = pt.y(); tt_support = patch->points[jj]; }
					if (pt.y() > bb) { bb = pt.y(); bb_support = patch->points[jj]; }
					//if (patch->road_class == PatchType::GUIDE)
					//	std::cerr << patch->points[jj].transpose() << std::endl;

				}
				patch->b_point[0] = direction_rotation * Vector3d(ll, bb, 0) + patch->mean;
				patch->b_point[1] = direction_rotation * Vector3d(rr, bb, 0) + patch->mean;
				patch->b_point[2] = direction_rotation * Vector3d(rr, tt, 0) + patch->mean;
				patch->b_point[3] = direction_rotation * Vector3d(ll, tt, 0) + patch->mean;

				Eigen::Matrix3d P0 = calcUncertainty(ipm, ll_support(0), ll_support(1));
				Eigen::Matrix3d P1 = calcUncertainty(ipm, rr_support(0), rr_support(1));
				Eigen::Matrix3d P2 = calcUncertainty(ipm, bb_support(0), bb_support(1));
				Eigen::Matrix3d P3 = calcUncertainty(ipm, tt_support(0), tt_support(1));
#ifdef DEBUG
				//cv::Mat mm(1000, 1000, CV_8UC3);
				//cv::circle(mm, cv::Point2f(ll_support(0), ll_support(1)), 5, cv::Scalar(255, 0, 0));
				//cv::circle(mm, cv::Point2f(rr_support(0), rr_support(1)), 5, cv::Scalar(0, 255, 0));
				//cv::circle(mm, cv::Point2f(bb_support(0), bb_support(1)), 5, cv::Scalar(0, 0, 255));
				//cv::circle(mm, cv::Point2f(tt_support(0), tt_support(1)), 5, cv::Scalar(255, 255, 0));

				////drawUncertainty(mm, ll_support, P0/config.IPM_RESO/config.IPM_RESO);
				////drawUncertainty(mm, rr_support, P1/config.IPM_RESO/config.IPM_RESO);
				////drawUncertainty(mm, bb_support, P2/config.IPM_RESO/config.IPM_RESO);
				////drawUncertainty(mm, tt_support, P3/config.IPM_RESO/config.IPM_RESO);
				//drawUncertainty_dist(mm, ll_support, P0 / config.IPM_RESO / config.IPM_RESO, Vector3d(-b_direction(1), b_direction(0), b_direction(2)));
				//drawUncertainty_dist(mm, rr_support, P1 / config.IPM_RESO / config.IPM_RESO, Vector3d(-b_direction(1), b_direction(0), b_direction(2)));
				//drawUncertainty_dist(mm, bb_support, P2 / config.IPM_RESO / config.IPM_RESO, b_direction);
				//drawUncertainty_dist(mm, tt_support, P3 / config.IPM_RESO / config.IPM_RESO, b_direction);


				//cv::imshow("mm_debug", mm);
				//cv::waitKey(0);
#endif

				Vector3d b_direction_t = Vector3d(-b_direction(1), b_direction(0), b_direction(2));
				patch->b_unc_dist[0] = sqrt(b_direction.transpose() * P2 * b_direction);
				patch->b_unc_dist[1] = sqrt(b_direction_t.transpose() * P1 * b_direction_t);
				patch->b_unc_dist[2] = sqrt(b_direction.transpose() * P3 * b_direction);
				patch->b_unc_dist[3] = sqrt(b_direction_t.transpose() * P0 * b_direction_t);
				if (patch->out_range)
				{
					patch->b_unc_dist[0] *=100;
					patch->b_unc_dist[1] *=100;
					patch->b_unc_dist[3] *=100;
				}
			}
			else
			{
				continue;
			}
		}

		if (patches_temp[i].road_class == PatchType::SOLID ||
			patches_temp[i].road_class == PatchType::DASHED ||
			patches_temp[i].road_class == PatchType::GUIDE ||
			patches_temp[i].road_class == PatchType::ZEBRA ||
			patches_temp[i].road_class == PatchType::STOP)
		{
			frame.patches[patches_temp[i].road_class].push_back(patch);
		}
	}
	return frame;
}

int RoadInstancePatchFrame::generateMetricPatches(const SensorConfig& config, const gv::IPMProcesser& ipm)
{
	auto cam = ipm._config;
	for (auto iter_class : patches)
	{
		for (int i = 0; i < iter_class.second.size(); i++)
		{
			auto& patch = iter_class.second[i];

			patch->points_metric.resize(patch->points.size());

			for (int j = 0; j < patch->points.size(); j++)
			{
				patch->points_metric[j] = cam.tic
					+ cam.Ric * ipm.IPM2Metric(cv::Point2f(patch->points[j].x(), patch->points[j].y()));
			}

			if (patch->line_valid)
			{
				patch->line_points_metric.resize(patch->line_points.size());
				for (int j = 0; j < patch->line_points.size(); j++)
				{
					patch->line_points_metric[j] = cam.tic
						+ cam.Ric * ipm.IPM2Metric(cv::Point2f(patch->line_points[j].x(), patch->line_points[j].y()));
				}
			}

			if (patch->road_class == PatchType::DASHED || patch->road_class == PatchType::GUIDE)
			{
				for (int ii = 0; ii < 4; ii++)
				{
					patch->b_point_metric[ii] = cam.tic
						+ cam.Ric * ipm.IPM2Metric(cv::Point2f(patch->b_point[ii].x(), patch->b_point[ii].y()));
				}
			}

			patch->mean_metric.setZero();
			Eigen::Vector3d vector_temp;

			for (int j = 0; j < patch->points_metric.size(); j++)
			{
				patch->mean_metric += patch->points_metric[j];
			}
			patch->mean_metric /= patch->points_metric.size();
			patch->percept_distance = patch->mean_metric(1);

			if (iter_class.first == PatchType::DASHED
				&& patch->h() > config.patch_dashed_min_h
				&& patch->h() < config.patch_dashed_max_h
				&& patch->mean_metric(1) < config.patch_dashed_max_dist)
				patch->valid_add_to_map = true;
			else if (iter_class.first == PatchType::GUIDE
				&& patch->h() > config.patch_guide_min_h
				&& patch->h() < config.patch_guide_max_h
				&& patch->mean_metric(1) < config.patch_guide_max_dist)
				patch->valid_add_to_map = true;
			else if (patch->line_valid)
				patch->valid_add_to_map = true;

		}
	}
	return 0;
}

double RoadInstancePatch::h() const
{
	return (b_point_metric[2] - b_point_metric[1]).norm();
}

double RoadInstancePatch::w() const
{
	return (b_point_metric[1] - b_point_metric[0]).norm();
}

Eigen::Vector3d RoadInstancePatch::d() const
{
		if (road_class == PatchType::DASHED || road_class == PatchType::GUIDE) 
			return (b_point_metric[2] - b_point_metric[1]).normalized();
		else if (road_class == PatchType::SOLID || road_class == PatchType::STOP)
		{
			assert(line_points_metric.size() >= 2);
			return (line_points_metric.back() - line_points_metric.front()).normalized();
		}
		else
			throw exception();
}

SensorConfig::SensorConfig(string path)
{
	gv::CameraConfig conf;

	cv::FileStorage fs_config(path, cv::FileStorage::READ);

	conf.IPM_HEIGHT = (int)fs_config["IPM_HEIGHT"];
	conf.IPM_WIDTH = (int)fs_config["IPM_WIDTH"];
	conf.IPM_RESO = (double)fs_config["IPM_RESO"];
	double t_start = (double)fs_config["t_start"];
	double t_end = (double)fs_config["t_end"];
	conf.RAW_RESIZE = 1;
	int pn = path.find_last_of('/');
	std::string configPath = path.substr(0, pn);

	conf.camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(configPath + '/' + (string)fs_config["cam0_calib"]);

	conf.cg = gv::CameraGroundGeometry((double)fs_config["cg_alpha"], (double)fs_config["cg_theta"], (double)fs_config["cg_h"]);

	cv::Mat cv_T;
	fs_config["body_T_cam0"] >> cv_T;
	Eigen::Matrix4d Tic;
	cv::cv2eigen(cv_T, Tic);
	conf.Ric = Tic.block<3, 3>(0, 0);
	conf.tic = Tic.block<3, 1>(0, 3);
	conf.Tic.setIdentity();
	conf.Tic.block<3, 3>(0, 0) = conf.Ric;
	conf.Tic.block<3, 1>(0, 3) = conf.tic;
	this->cam = conf;
	this->pose_smooth_window = (int)fs_config["pose_smooth_window"];
	this->need_smooth = (bool)(int)fs_config["need_smooth"];
	this->large_slope_thresold = (double)fs_config["large_slope_thresold"];

	this->patch_min_size = (int)fs_config["patch.min_size"];
	this->patch_dashed_min_h = (double)fs_config["patch.dashed_min_h"];
	this->patch_dashed_max_h = (double)fs_config["patch.dashed_max_h"];
	this->patch_dashed_max_dist = (double)fs_config["patch.dashed_max_dist"];
	this->patch_guide_min_h = (double)fs_config["patch.guide_min_h"];
	this->patch_guide_max_h = (double)fs_config["patch.guide_max_h"];
	this->patch_guide_max_dist = (double)fs_config["patch.guide_max_dist"];
	this->patch_solid_max_dist = (double)fs_config["patch.solid_max_dist"];
	this->patch_stop_max_dist = (double)fs_config["patch.stop_max_dist"];

	this->mapping_step = (int)fs_config["mapping.step"];
	this->mapping_patch_freeze_distance = (double)fs_config["mapping.patch_freeze_distance"];
	this->mapping_line_freeze_distance = (double)fs_config["mapping.line_freeze_distance"];
	this->mapping_line_freeze_max_length = (double)fs_config["mapping.line_freeze_max_length"];
	this->mapping_line_cluster_max_dist = (double)fs_config["mapping.line_cluster_max_dist"];
	this->mapping_line_cluster_max_across_dist1 = (double)fs_config["mapping.line_cluster_max_across_dist1"];
	this->mapping_line_cluster_max_across_dist2 = (double)fs_config["mapping.line_cluster_max_across_dist2"];
	this->mapping_line_cluster_max_theta = (double)fs_config["mapping.line_cluster_max_theta"];

	this->enable_vis_image = (bool)(int)fs_config["enable_vis_image"];
	this->enable_vis_3d = (bool)(int)fs_config["enable_vis_3d"];

	this->t_start = (double)fs_config["t_start"];
	this->t_end = (double)fs_config["t_end"];

}
