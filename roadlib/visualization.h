#pragma once
#include "roadlib.h"
#include "gviewer.h"
#include <fstream>

int visualize_roadmap(const RoadInstancePatchMap& road_map,
	vector<VisualizedInstance>& vis_instances);

int visualize_vehicle(const Vector3d enu, const Vector3d att,
	vector<VisualizedInstance>& vis_instances);

int visualize_marker(const Vector3d enu, float r, float g, float b, vector<VisualizedInstance>& vis_instances);

void genColorLabel(cv::Mat ipm_single, cv::Mat ipm_color);
