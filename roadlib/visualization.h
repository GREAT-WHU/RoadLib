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
