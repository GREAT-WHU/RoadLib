#include "visualization.h"

int visualize_roadmap(const RoadInstancePatchMap& road_map,
	vector<VisualizedInstance>& vis_instances)
{
	for (auto iter_class = road_map.patches.begin(); iter_class != road_map.patches.end(); iter_class++)
	{
		for (auto iter_instance = iter_class->second.begin(); iter_instance != iter_class->second.end(); iter_instance++)
		{
			if ((*iter_instance)->valid_add_to_map)
			{
				if ((*iter_instance)->line_valid)
				{
					VisualizedInstance vis_instance;
					vis_instance.type = VisualizedPatchType::LINE_SEGMENT;
					vis_instance.color[0] = 0.0 / 255.0;
					vis_instance.color[1] = 128.0 * 1.25 / 255.0;
					vis_instance.color[2] = 0.0 / 255.0;
					Vector3d color0 = Vector3d(1, 0, 0);
					Vector3d color1 = Vector3d(1, 1, 0);
					Vector3d color2 = Vector3d(0, 1, 0);
					for (int ii = 0; ii < (*iter_instance)->line_points_metric.size(); ii++)
					{
						if ((*iter_instance)->road_class == PatchType::SOLID)
						{
							vis_instance.pts.push_back((*iter_instance)->line_points_metric[ii]);
							double point_error = sqrt((*iter_instance)->line_points_uncertainty[ii](0, 0) + (*iter_instance)->line_points_uncertainty[ii](1, 1));
							if (point_error > 0.5)
								vis_instance.pts_color.push_back((point_error - 1) / (1 - 0.5) * color0 +
									(1 - (point_error - 1) / (1 - 0.5)) * color1);
							else
								vis_instance.pts_color.push_back((point_error - 0.1) / (0.5 - 0.1) * color1 +
									(1 - (point_error - 0.1) / (0.5 - 0.1)) * color2);
						}
						else if ((*iter_instance)->road_class == PatchType::STOP)
						{
							vis_instance.pts.push_back((*iter_instance)->line_points_metric[ii]);
							vis_instance.pts_color.push_back(Vector3d(1, 0, 0));
						}
					}
					if (vis_instance.pts.size() >= 2)
					{
						vis_instances.push_back(vis_instance);
					}

					if ((*iter_instance)->merged)
					{
						for (int ii = 0; ii < (*iter_instance)->line_points_metric.size(); ii++)
						{
							VisualizedInstance vis_instance;
							vis_instance.type = VisualizedPatchType::BOX;
							if ((*iter_instance)->road_class == PatchType::SOLID)
							{
								vis_instance.color[0] = 0.0 / 255.0;
								vis_instance.color[1] = 128.0 * 1.25 / 255.0;
								vis_instance.color[2] = 0.0 / 255.0;
							}
							else if ((*iter_instance)->road_class == PatchType::STOP)
							{
								vis_instance.color[0] = 128.0 * 1.25 / 255.0;
								vis_instance.color[1] = 0.0 / 255.0;
								vis_instance.color[2] = 0.0 / 255.0;
							}
							vis_instance.t = (*iter_instance)->line_points_metric[ii];
							vis_instance.R = Eigen::Matrix3d::Identity();
							vis_instance.h = 0.2;
							vis_instance.w = 0.2;
							vis_instance.l = 0.2;
							vis_instances.push_back(vis_instance);
						}
					}
				}
				else
				{
					//double instance_yaw = atan2((*iter_instance)->direction_metric(1), (*iter_instance)->direction_metric(0));
					double dy = ((*iter_instance)->b_point_metric[3] - (*iter_instance)->b_point_metric[0]).norm();
					double dx = ((*iter_instance)->b_point_metric[1] - (*iter_instance)->b_point_metric[0]).norm();


					double instance_yaw = atan2((*iter_instance)->b_point_metric[2].y() - (*iter_instance)->b_point_metric[0].y(),
						(*iter_instance)->b_point_metric[2].x() - (*iter_instance)->b_point_metric[0].x());
					Vector3d mean_box = ((*iter_instance)->b_point_metric[0]
						+ (*iter_instance)->b_point_metric[1]
						+ (*iter_instance)->b_point_metric[2]
						+ (*iter_instance)->b_point_metric[3]) / 4.0;

					VisualizedInstance vis_instance;
					vis_instance.type = VisualizedPatchType::LINE_SEGMENT;
					for (int ii = 0; ii < 4; ii++)
					{
						vis_instance.pts.push_back((*iter_instance)->b_point_metric[ii]);
						if ((*iter_instance)->road_class == PatchType::DASHED)
							vis_instance.pts_color.push_back(Vector3d(128.0 * 1.25 / 255.0, 128.0 * 1.25 / 255.0, 0.0 / 255.0));
						else
							vis_instance.pts_color.push_back(Vector3d(0.0 * 1.25 / 255.0, 0 * 1.25 / 255.0, 128.0 * 1.25 / 255.0));

					}
					vis_instances.push_back(vis_instance);
				}
			}
		}
	}

	return 0;
}

int visualize_vehicle(const Vector3d enu, const Vector3d att,
	vector<VisualizedInstance>& vis_instances)
{
	VisualizedInstance vis_instance_vehicle;
	vis_instance_vehicle.type = VisualizedPatchType::BOX;
	vis_instance_vehicle.color[0] = 0.0;
	vis_instance_vehicle.color[1] = 0.0;
	vis_instance_vehicle.color[2] = 0.0;
	vis_instance_vehicle.t = enu;
	vis_instance_vehicle.R = a2mat(att);
	vis_instance_vehicle.h = 1.753;
	vis_instance_vehicle.l = 1.849;
	vis_instance_vehicle.w = 4.690;
	vis_instances.push_back(vis_instance_vehicle);
	return 0;

}

int visualize_marker(const Vector3d enu, float r, float g, float b, vector<VisualizedInstance>& vis_instances)
{
	VisualizedInstance vis_instance_marker;
	vis_instance_marker.type = VisualizedPatchType::BOX;
	vis_instance_marker.color[0] = r;
	vis_instance_marker.color[1] = g;
	vis_instance_marker.color[2] = b;
	vis_instance_marker.t = enu;
	vis_instance_marker.R = Eigen::Matrix3d::Identity();
	vis_instance_marker.h = 0.3;
	vis_instance_marker.l = 0.3;
	vis_instance_marker.w = 0.3;
	vis_instances.push_back(vis_instance_marker);
	return 0;

}

void genColorLabel(cv::Mat ipm_single, cv::Mat ipm_color)
{
	for (int i = 0; i < ipm_single.rows; i++)
		for (int j = 0; j < ipm_single.cols; j++)
		{
			if (ipm_single.at<uchar>(i, j) == 1) ipm_color.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255, 255);
			if (ipm_single.at<uchar>(i, j) == 2) ipm_color.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 128, 0);
			if (ipm_single.at<uchar>(i, j) == 3) ipm_color.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 0, 128);
			if (ipm_single.at<uchar>(i, j) == 4) ipm_color.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 128, 0);
			if (ipm_single.at<uchar>(i, j) == 5) ipm_color.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
		}
}
