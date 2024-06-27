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
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
map<PatchType, pcl::PointCloud<pcl::PointXYZ>::Ptr> class_pts;
map<PatchType, vector<pair<int, int>>> class_pts_index;
map<PatchType, pcl::KdTreeFLANN<pcl::PointXYZ>> class_kdtree;

int RoadInstancePatchMap::addFrame(const RoadInstancePatchFrame& frame)
{
	for (auto iter_class = frame.patches.begin(); iter_class != frame.patches.end(); iter_class++)
	{
		for (auto iter_instance = iter_class->second.begin(); iter_instance != iter_class->second.end(); iter_instance++)
		{
			if ((*iter_instance)->valid_add_to_map)
			{
				patches[iter_class->first].push_back(make_shared<RoadInstancePatch>(RoadInstancePatch()));
				auto& patch = patches[iter_class->first].back();
				patch->frame_id = frame.id;
				patch->road_class = (*iter_instance)->road_class;
				patch->mean_metric = frame.t + frame.R * (*iter_instance)->mean_metric;
				patch->valid_add_to_map = true;
				patch->line_valid = (*iter_instance)->line_valid;
				patch->percept_distance = (*iter_instance)->percept_distance;
				patch->mean_uncertainty = frame.R * (*iter_instance)->mean_uncertainty * frame.R.transpose();
				patch->line_points_uncertainty = (*iter_instance)->line_points_uncertainty;
				if ((*iter_instance)->line_valid)
				{
					patch->line_points_metric.resize((*iter_instance)->line_points_metric.size());
					patch->line_points_uncertainty.resize((*iter_instance)->line_points_metric.size());
					for (int j = 0; j < patch->line_points_metric.size(); j++)
					{
						patch->line_points_metric[j] = frame.t + frame.R * (*iter_instance)->line_points_metric[j];
						patch->line_points_uncertainty[j] = frame.R * patch->line_points_uncertainty[j] * frame.R.transpose();
					}
				}
				for (int ii = 0; ii < 4; ii++)
				{
					patch->b_point_metric[ii] = frame.t + frame.R * (*iter_instance)->b_point_metric[ii];
					patch->b_unc_dist[ii] = (*iter_instance)->b_unc_dist[ii];
				}
			}

		}
	}

	queued_poses[frame.id] = make_pair(frame.R, frame.t);
	timestamps[frame.id] = frame.time;
	return 0;
}

int RoadInstancePatchMap::clearMap()
{
	patches.clear();
	return 0;
}

int RoadInstancePatchMap::saveMapToFileBinaryRaw(string filename)
{
	std::cout<<"[INFO] Saving map to "<<filename<<"..."<<std::endl;
	size_t count;
	float float_buffer[9];
	FILE* fp = fopen(filename.c_str(), "wb");
	fwrite(ref.data(), sizeof(double), 3, fp);
	count = patches.size();
	fwrite(&count, sizeof(count), 1, fp);
	for (auto iter = patches.begin(); iter != patches.end(); iter++)
	{
		//if (iter->first != PatchType::SOLID) continue;
		fwrite(&(iter->first), sizeof(iter->first), 1, fp);
		count = iter->second.size();
		fwrite(&(count), sizeof(count), 1, fp);
		for (int i = 0; i < iter->second.size(); i++)
		{
			auto& this_patch = iter->second[i];
			fwrite(&(RoadInstancePatchFrame::next_id), sizeof(RoadInstancePatchFrame::next_id), 1, fp);
			fwrite(&(this_patch->id), sizeof(this_patch->id), 1, fp);
			fwrite(&(this_patch->road_class), sizeof(this_patch->road_class), 1, fp);

			fwrite(&(this_patch->line_valid), sizeof(this_patch->line_valid), 1, fp);
			fwrite(&(this_patch->frozen), sizeof(this_patch->frozen), 1, fp);
			fwrite(&(this_patch->merged), sizeof(this_patch->merged), 1, fp);
			fwrite(&(this_patch->valid_add_to_map), sizeof(this_patch->valid_add_to_map), 1, fp);

			for (int z = 0; z < 3; z++) float_buffer[z] = (float)this_patch->mean_metric(z);
			fwrite(float_buffer, sizeof(float), 3, fp);

			for (int iii = 0; iii < 4; iii++)
			{
				for (int z = 0; z < 3; z++) float_buffer[z] = (float)this_patch->b_point_metric[iii](z);
				fwrite(float_buffer, sizeof(float), 3, fp);
			}

			for (int z = 0; z < 4; z++) float_buffer[z] = (float)this_patch->b_unc_dist[z];
			fwrite(float_buffer, sizeof(float), 4, fp);



			count = this_patch->line_points_metric.size();
			fwrite(&(count), sizeof(count), 1, fp);
			for (int ii = 0; ii < this_patch->line_points_metric.size(); ii++)
			{
				for (int z = 0; z < 3; z++) float_buffer[z] = (float)this_patch->line_points_metric[ii](z);
				fwrite(float_buffer, sizeof(float), 3, fp);
			}

			for (int z = 0; z < 9; z++) float_buffer[z] = (float)this_patch->mean_uncertainty(z / 3, z % 3);
			fwrite(float_buffer, sizeof(float), 9, fp);

			count = this_patch->line_points_uncertainty.size();
			fwrite(&(count), sizeof(count), 1, fp);
			for (int ii = 0; ii < this_patch->line_points_uncertainty.size(); ii++)
			{
				for (int z = 0; z < 9; z++) float_buffer[z] = (float)this_patch->line_points_uncertainty[ii](z / 3, z % 3);
				fwrite(float_buffer, sizeof(float), 9, fp);
			}
			fwrite(&(this_patch->percept_distance), sizeof(this_patch->percept_distance), 1, fp);
		}
	}
	fclose(fp);
	std::cout<<"[INFO] Finished."<<std::endl;
	return 0;
}

int RoadInstancePatchMap::loadMapFromFileBinaryRaw(string filename)
{
	std::cout<<"[INFO] Loading map from "<<filename<<"..."<<std::endl;
	FILE* fp = fopen(filename.c_str(), "rb");
	for (int i = 0; i < 3; i++) fread(&ref(i), sizeof(double), 1, fp);

	size_t class_count; fread(&class_count, sizeof(size_t), 1, fp);
	float float_temp[9];
	for (int i_class = 0; i_class < class_count; i_class++)
	{
		PatchType road_class;
		fread(&road_class, sizeof(int), 1, fp);
		size_t pcount;
		fread(&pcount, sizeof(size_t), 1, fp);
		patches[road_class];
		for (int i_patch = 0; i_patch < pcount; i_patch++)
		{
			RoadInstancePatch patch;
			fread(&patch.next_id, sizeof(patch.next_id), 1, fp);
			fread(&patch.id, sizeof(patch.id), 1, fp);
			fread(&patch.road_class, sizeof(patch.road_class), 1, fp);
			fread(&patch.line_valid, sizeof(patch.line_valid), 1, fp);
			fread(&(patch.frozen), sizeof(patch.frozen), 1, fp);
			fread(&(patch.merged), sizeof(patch.merged), 1, fp);
			fread(&patch.valid_add_to_map, sizeof(patch.valid_add_to_map), 1, fp);
			fread(&float_temp[0], sizeof(float), 3, fp);
			for (int i = 0; i < 3; i++) patch.mean_metric(i) = float_temp[i];

			for (int iii = 0; iii < 4; iii++)
			{
				fread(&float_temp[0], sizeof(float), 3, fp);
				for (int i = 0; i < 3; i++) patch.b_point_metric[iii](i) = float_temp[i];
			}
			fread(&float_temp[0], sizeof(float), 4, fp);
			for (int iii = 0; iii < 4; iii++)
				patch.b_unc_dist[iii] = float_temp[iii];

			size_t lpcount;

			fread(&lpcount, sizeof(lpcount), 1, fp);
			patch.line_points_metric.resize(lpcount);
			for (int lp = 0; lp < lpcount; lp++)
			{
				fread(&float_temp[0], sizeof(float), 3, fp);
				for (int i = 0; i < 3; i++) patch.line_points_metric[lp](i) = float_temp[i];
			}
			fread(&float_temp[0], sizeof(float), 9, fp);
			for (int i = 0; i < 9; i++) patch.mean_uncertainty(i / 3, i % 3) = float_temp[i];

			fread(&lpcount, sizeof(lpcount), 1, fp);
			patch.line_points_uncertainty.resize(lpcount);
			for (int lp = 0; lp < lpcount; lp++)
			{
				fread(&float_temp[0], sizeof(float), 9, fp);
				for (int i = 0; i < 9; i++) patch.line_points_uncertainty[lp](i / 3, i % 3) = float_temp[i];
			}

			fread(&patch.percept_distance, sizeof(patch.percept_distance), 1, fp);
			patches[road_class].push_back(make_shared<RoadInstancePatch>(patch));
		}
	}
	std::cout << "[INFO] Map successfully loaded." << std::endl;
	return 0;
}

int RoadInstancePatchMap::buildKDTree()
{
	for (auto iter_class = this->patches.begin(); iter_class != this->patches.end(); iter_class++)
	{
		class_pts.emplace(iter_class->first, pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
		class_pts_index.emplace(iter_class->first, vector<pair<int, int>>());
		class_kdtree.emplace(iter_class->first, pcl::KdTreeFLANN<pcl::PointXYZ>());
		auto& this_pts_ptr = class_pts[iter_class->first];
		auto& kdtree = class_kdtree[iter_class->first];
		auto& this_pts_index = class_pts_index[iter_class->first];
		if (iter_class->first == PatchType::DASHED || iter_class->first == PatchType::GUIDE)
		{
			for (int i = 0; i < iter_class->second.size(); i++)
			{
				this_pts_index.push_back(make_pair(i, 0));
			}
		}
		else if (iter_class->first == PatchType::SOLID || iter_class->first == PatchType::STOP)
		{
			for (int i = 0; i < iter_class->second.size(); i++)
			{
				for (int j = 0; j < iter_class->second[i]->line_points_metric.size(); j++)
				{
					this_pts_index.push_back(make_pair(i, j));
				}
			}
		}

		this_pts_ptr->width = this_pts_index.size();
		this_pts_ptr->height = 1;
		this_pts_ptr->is_dense = false;
		this_pts_ptr->points.resize(this_pts_ptr->width * this_pts_ptr->height);

		if (iter_class->first == PatchType::DASHED || iter_class->first == PatchType::GUIDE)
		{
			for (size_t i = 0; i < this_pts_index.size(); i++)
			{
				this_pts_ptr->points[i].x = iter_class->second[this_pts_index[i].first]->mean_metric(0);
				this_pts_ptr->points[i].y = iter_class->second[this_pts_index[i].first]->mean_metric(1);
				this_pts_ptr->points[i].z = iter_class->second[this_pts_index[i].first]->mean_metric(2);
			}
		}
		else if (iter_class->first == PatchType::SOLID || iter_class->first == PatchType::STOP)
		{
			for (size_t i = 0; i < this_pts_index.size(); i++)
			{
				this_pts_ptr->points[i].x = iter_class->second[this_pts_index[i].first]->line_points_metric[this_pts_index[i].second](0);
				this_pts_ptr->points[i].y = iter_class->second[this_pts_index[i].first]->line_points_metric[this_pts_index[i].second](1);
				this_pts_ptr->points[i].z = iter_class->second[this_pts_index[i].first]->line_points_metric[this_pts_index[i].second](2);
			}
		}

		kdtree.setInputCloud(this_pts_ptr);
	}

	return 0;
}

map<PatchType, vector<pair<int, int>>> RoadInstancePatchMap::mapMatch(RoadInstancePatchFrame& frame, int mode)
{
	bool wide_search = true;
	double search_radius = wide_search ? 20 : 10;
	map<PatchType, vector<pair<int, int>>> match_pairs;
	for (auto iter_class = frame.patches.begin(); iter_class != frame.patches.end(); iter_class++)
	{
		match_pairs[iter_class->first] = vector<pair<int, int>>();
		if (iter_class->first == PatchType::DASHED || iter_class->first == PatchType::GUIDE
			|| iter_class->first == PatchType::SOLID || iter_class->first == PatchType::STOP)
		{
			for (int i = 0; i < iter_class->second.size(); i++)
			{
				auto& this_patch = iter_class->second[i];

				Vector3d mean_metric = frame.t + frame.R * this_patch->mean_metric;

				vector<int> indice2;
				vector<float> dist2;
				class_kdtree[iter_class->first].radiusSearch(pcl::PointXYZ((float)mean_metric(0), (float)mean_metric(1), 0.0), search_radius, indice2, dist2);


				if (indice2.size() > 0)
				{
					if (iter_class->first == PatchType::DASHED || iter_class->first == PatchType::GUIDE)
					{
						if (fabs(this->patches[iter_class->first][class_pts_index[iter_class->first][indice2[0]].first]->h() - frame.patches[iter_class->first][i]->h()) > 2.0
							|| fabs(this->patches[iter_class->first][class_pts_index[iter_class->first][indice2[0]].first]->w() - frame.patches[iter_class->first][i]->w()) > 2.0)
							continue;
						Vector3d n1 = this->patches[iter_class->first][class_pts_index[iter_class->first][indice2[0]].first]->mean_metric;
						Vector3d n2 = frame.patches[iter_class->first][i]->mean_metric;
						double cos_theta = fabs(n1.dot(n2) / (n1.norm() * n2.norm()));

						//if (cos_theta > cos(15.0 / 180 * M_PI))
						//	continue;
					}
					if (mode == 1) // strict
					{
						if (iter_class->first == PatchType::DASHED || iter_class->first == PatchType::GUIDE)
						{
							Vector3d n1 = this->patches[iter_class->first][class_pts_index[iter_class->first][indice2[0]].first]->mean_metric;
							Vector3d n2 = frame.patches[iter_class->first][i]->mean_metric;
							double cos_theta = fabs(n1.dot(n2) / (n1.norm() * n2.norm()));

							if ((this->patches[iter_class->first][class_pts_index[iter_class->first][indice2[0]].first]->mean_metric - mean_metric).norm() > 1.0
								|| this->patches[iter_class->first][class_pts_index[iter_class->first][indice2[0]].first]->h() < 1.5)
								continue;
						}
					}
					match_pairs[iter_class->first].push_back(make_pair(class_pts_index[iter_class->first][indice2[0]].first, i));
				}
			}
		}
	}

	return match_pairs;
}

vector<pair<int, int>> RoadInstancePatchMap::getLineMatch(RoadInstancePatchFrame& frame, PatchType road_class, int frame_line_count, int map_line_count, int mode)
{
	auto& this_patch_frame = frame.patches[road_class][frame_line_count];
	auto& this_patch_map = this->patches[road_class][map_line_count];

	vector<pair<int, int>> match_pairs;
	Vector3d last_w_p = Vector3d(0, 0, 0);

	for (int i = 0; i < this_patch_frame->line_points_metric.size(); i++)
	{
		Vector3d w_p = frame.R * this_patch_frame->line_points_metric[i] + frame.t;
		if (road_class == PatchType::SOLID)
		{
			if ((w_p - last_w_p).norm() < 3.0) continue;
		}
		map<double, int> dist;
		for (int j = 0; j < this_patch_map->line_points_metric.size(); j++)
		{
			dist.emplace((w_p - this_patch_map->line_points_metric[j]).norm(), j);
		}
		auto iter_dist = dist.begin();
		auto iter_dist_2 = dist.begin(); iter_dist_2++;

		int match_id = min(iter_dist->second, iter_dist_2->second);

		if (match_id == 0 || match_id > this_patch_map->line_points_metric.size() - 3)
		{
			continue;
		}
		last_w_p = w_p;
		Vector3d w_p0 = this_patch_map->line_points_metric[match_id];
		Vector3d w_p1 = this_patch_map->line_points_metric[match_id + 1];

		double Dxlj = w_p1[0] - w_p0[0];
		double Dylj = w_p1[1] - w_p0[1];
		double Dzlj = w_p1[2] - w_p0[2];

		double Dxli = w_p[0] - w_p0[0];
		double Dyli = w_p[1] - w_p0[1];
		double Dzli = w_p[2] - w_p0[2];

		double a = Dylj * Dzli - Dzlj * Dyli;
		double b = Dzlj * Dxli - Dxlj * Dzli;
		double c = Dxlj * Dyli - Dylj * Dxli;

		double Dvljxvli = pow((a * a + b * b + c * c), 0.5);
		double Dvlj = pow(Dxlj * Dxlj + Dylj * Dylj + Dzlj * Dzlj, 0.5);
		double ddd = Dvljxvli / Dvlj;
		if (mode > 0 && ddd > 1.0) continue;
		match_pairs.push_back(make_pair(i, match_id));
	}

	return match_pairs;
}

int RoadInstancePatchMap::unfreeze()
{
	for (auto iter_class = patches.begin(); iter_class != patches.end(); iter_class++)
		for (int i = 0; i < iter_class->second.size(); i++)
		{
			iter_class->second[i]->frozen = false;
		}
	return 0;
}

int RoadInstancePatchMap::mergeMap(const RoadInstancePatchMap& road_map_other)
{
	auto refp0 = this->ref;
	auto refp1 = road_map_other.ref;
	assert(refp0.norm() > 6000e3 && refp0.norm() < 7000e3);
	assert(refp1.norm() > 6000e3 && refp1.norm() < 7000e3);
	//Eigen::Matrix3d Ren0 = calcRne(refp0.pos);
	//Eigen::Vector3d ten0 = refp0.pos;
	//Eigen::Matrix3d Ren1 = calcRne(refp1.pos);
	//Eigen::Vector3d ten1 = refp1.pos;
	long long next_id = -1;
	for (auto iter_class = this->patches.begin(); iter_class != this->patches.end(); iter_class++)
	{
		for (int i = 0; i < iter_class->second.size(); i++)
		{
			if (iter_class->second[i]->id > next_id)
				next_id = iter_class->second[i]->id;
		}
	}
	next_id++;

	Matrix3d Rn0e = calcRne(refp0);
	Matrix3d Rn1e = calcRne(refp1);
	for (auto iter_class = road_map_other.patches.begin(); iter_class != road_map_other.patches.end(); iter_class++)
	{
		for (int i = 0; i < iter_class->second.size(); i++)
		{
			auto& this_patch = iter_class->second[i];
			auto patch_copy = this_patch;
			patch_copy->id += next_id;
			// Assuming $Rn0e \sim Rn1e$, the uncertainties wouldn't be changed.
			for (int ii = 0; ii < 4; ii++)
				patch_copy->b_point_metric[ii] = Rn0e * (Rn1e.transpose() * patch_copy->b_point_metric[ii] + refp1 - refp0);
			for (int ii = 0; ii < patch_copy->line_points_metric.size(); ii++)
				patch_copy->line_points_metric[ii] = Rn0e * (Rn1e.transpose() * patch_copy->line_points_metric[ii] + refp1 - refp0);
			patch_copy->mean_metric = Rn0e * (Rn1e.transpose() * patch_copy->mean_metric + refp1 - refp0);
			this->patches[iter_class->first].push_back(patch_copy);
		}
	}
	return 0;
}

int RoadInstancePatchMap::cleanMap()
{
	for (auto iter_class = patches.begin(); iter_class != patches.end(); iter_class++)
	{
		for (auto iter_patch = iter_class->second.begin(); iter_patch != iter_class->second.end();)
		{
			bool bad_flag = false;

			if (iter_class->first == PatchType::SOLID || iter_class->first == PatchType::STOP)
			{
				for (int i = 0; i < (*iter_patch)->line_points_metric.size(); i++)
				{
					if (isnan((*iter_patch)->line_points_metric[i].norm()))
					{
						bad_flag = true;
					}
				}
				if ((*iter_patch)->line_points_metric.size() < 5 || ((*iter_patch)->line_points_metric.back() - (*iter_patch)->line_points_metric.front()).norm() < 5)
					bad_flag = true;

			}
			else if (iter_class->first == PatchType::DASHED || iter_class->first == PatchType::GUIDE)
			{
				if (isnan((*iter_patch)->b_point_metric[0].norm()) ||
					isnan((*iter_patch)->b_point_metric[1].norm()) ||
					isnan((*iter_patch)->b_point_metric[2].norm()) ||
					isnan((*iter_patch)->b_point_metric[3].norm()) ||
					isnan((*iter_patch)->mean_metric.norm())||
					(*iter_patch)->w() < 0.01 || (*iter_patch)->h() < 0.01)
				{
					bad_flag = true;
				}
			}
			if (bad_flag)
				iter_patch = iter_class->second.erase(iter_patch);
			else
			{
				iter_patch++;
			}
		}
	}

	return 0;
}

int RoadInstancePatchMap::geoRegister(const Trajectory& new_traj, vector<VisualizedInstance>& lines_vis)
{
	std::cerr<<"[INFO] Start geo-registering."<<std::endl;
	VisualizedInstance vis_instance_template;
	vis_instance_template.type = VisualizedPatchType::LINE_SEGMENT;
	vis_instance_template.pts = vector<Vector3d>(2);
	vis_instance_template.pts_color = vector<Vector3d>(2, Eigen::Vector3d(0, 0, 1.0));
	vis_instance_template.alpha = 0.25f;
	vis_instance_template.linewidth = 0.5f;

	ref = new_traj.ref;

	map<long long, pair<Matrix3d, Vector3d>> old_poses = queued_poses;
	map<long long, pair<Matrix3d, Vector3d>> new_poses;
	for (auto iter = timestamps.begin(); iter != timestamps.end(); iter++)
	{
		auto iiter = new_traj.poses.lower_bound(iter->second - 0.001);
		if (fabs(iiter->first - iter->second) < 0.01)
			new_poses[iter->first] = make_pair(iiter->second.R, iiter->second.t);
	}

	for (auto iter_class = patches.begin(); iter_class != patches.end(); iter_class++)
	{
		auto start = chrono::system_clock::now();
		for (int i = 0; i < iter_class->second.size(); i++)
		{
			auto& this_patch = iter_class->second[i];
			if (!(this_patch->frozen)) continue; // TODO!!!!!!!!!!!!!!!

			if (iter_class->first == PatchType::SOLID || iter_class->first == PatchType::STOP)
			{
				if (this_patch->linked_frames.size() != this_patch->line_points_metric.size()) continue;

				vector<Eigen::Vector3d*> pts;
				for (int j = 0; j < this_patch->line_points_metric.size(); j++)
					pts.push_back(&this_patch->line_points_metric[j]);

				for (int j = 0; j < pts.size(); j++)
				{
					Vector3d twf_count; twf_count.setZero();
					int ccount = 0;
					for (int iii = 0; iii < this_patch->linked_frames[j].size(); iii++)
					{
						long long id = this_patch->linked_frames[j][iii];
						Eigen::Vector3d tvf = old_poses[id].first.transpose() * (*pts[j] - old_poses[id].second);
						Eigen::Vector3d twf = new_poses[id].first * tvf + new_poses[id].second;
						twf_count += twf;
						ccount += 1;

						vis_instance_template.pts[0] = *pts[j];
						vis_instance_template.pts[1] = old_poses[id].second;
						lines_vis.push_back(vis_instance_template);
					}
					*pts[j] = twf_count / ccount;
				}
			}
			else
			{
				if (this_patch->linked_frames.size() == 0) continue;
				vector<Eigen::Vector3d*> pts;

				Vector3d twf_count;
				int ccount;

				twf_count.setZero();
				ccount = 0;
				for (int iii = 0; iii < this_patch->linked_frames[0].size(); iii++)
				{
					long long id = this_patch->linked_frames[0][iii];
					Eigen::Vector3d tvf = old_poses[id].first.transpose() * (this_patch->mean_metric - old_poses[id].second);
					Eigen::Vector3d twf = new_poses[id].first * tvf + new_poses[id].second;
					twf_count += twf;
					ccount += 1;

					vis_instance_template.pts[0] = this_patch->mean_metric;
					vis_instance_template.pts[1] = old_poses[id].second;
					lines_vis.push_back(vis_instance_template);
				}
				this_patch->mean_metric = twf_count / ccount;

				for (int j = 0; j < 4; j++)
				{
					twf_count.setZero();
					ccount = 0;
					for (int iii = 0; iii < this_patch->linked_frames[0].size(); iii++)
					{
						long long id = this_patch->linked_frames[0][iii];
						Eigen::Vector3d tvf = old_poses[id].first.transpose() * (this_patch->b_point_metric[j] - old_poses[id].second);
						Eigen::Vector3d twf = new_poses[id].first * tvf + new_poses[id].second;
						//if (iter_class->first == PatchType::DASHED) std::cerr <<j<<" "<< new_poses[id].second.transpose() << std::endl;
						twf_count += twf;
						ccount += 1;

						vis_instance_template.pts[0] = this_patch->b_point_metric[j];
						vis_instance_template.pts[1] = old_poses[id].second;
						lines_vis.push_back(vis_instance_template);
					}
					this_patch->b_point_metric[j] = twf_count / ccount;

				}
			}
		}
	}
	return 0;
}


int RoadInstancePatchMap::mergePatches(const SensorConfig& config, const int mode, const Eigen::Matrix3d& Rwv, const Eigen::Vector3d& twv)
{
	map<PatchType, vector<shared_ptr<RoadInstancePatch>>> patches_new;

	// clear redundant patches
	if (ignore_frame_ids.size() > 0)
		for (auto iter_class = patches.begin(); iter_class != patches.end(); iter_class++)
		{
			for (auto iter_patch = iter_class->second.begin(); iter_patch != iter_class->second.end();)
			{
				if (ignore_frame_ids.find((*iter_patch)->frame_id) != ignore_frame_ids.end())
				{
					if (iter_class->first == PatchType::SOLID)
						iter_patch++;
					//iter_patch = iter_class->second.erase(iter_patch);
					else if (iter_class->first == PatchType::DASHED || iter_class->first == PatchType::GUIDE || iter_class->first == PatchType::STOP)
					{
						if ((*iter_patch)->percept_distance > ignore_frame_ids[(*iter_patch)->frame_id] + 0.5)
						{
							iter_patch = iter_class->second.erase(iter_patch);
						}
						else
						{
							std::cout << "[INFO] Catch a safe patch!" << std::endl;
							iter_patch++;
						}
					}
				}
				else
				{
					iter_patch++;
				}
			}
		}
	ignore_frame_ids.clear();

	// start patch clustering and merging
	for (auto iter_class = patches.begin(); iter_class != patches.end(); iter_class++)
	{
		auto start = chrono::system_clock::now();
		vector<cv::Point2f> pts;
		vector<int> cluster_flag(iter_class->second.size(), -1);
		map<int, vector<int>> cluster_index;
		int cluster_id = 0;

		for (int i = 0; i < iter_class->second.size(); i++)
		{
			auto& iter_instance = iter_class->second[i];
			pts.push_back(cv::Point2f((iter_instance)->mean_metric(0),
				(iter_instance)->mean_metric(1)));
		}
		cv::Mat pts_vec = cv::Mat(pts).reshape(1);
		pts_vec.convertTo(pts_vec, CV_32F);
		cv::flann::KDTreeIndexParams indexParams(2);
		cv::flann::Index kdtree(pts_vec, indexParams);
		long long comp_count = 0;

		if (iter_class->first == PatchType::DASHED || iter_class->first == PatchType::GUIDE)
		{
			for (int i = 0; i < iter_class->second.size(); i++)
			{
				if (!iter_class->second[i]->valid_add_to_map) continue;
				if (cluster_flag[i] == -1)
				{
					vector<int> indice(1000);
					vector<float> dist(1000);
					//int num = kdtree.radiusSearch(vector<float>({ pts[i].x,pts[i].y }), indice, dist, 20.0, 1000);
					int num = pts.size() > 500 ? 500 : pts.size();
					kdtree.knnSearch(vector<float>({ pts[i].x,pts[i].y }), indice, dist, num);

					vector<int> intersection_flag(num, -1);
					queue<int> new_intersection_instance; new_intersection_instance.push(0);
					vector<int> same_cluster_flag; // for bi-directional mathching

					if (iter_class->second[i]->frozen)
					{
						intersection_flag[0] = 1;
					}
					else
					{
						while (new_intersection_instance.size() > 0)
						{
							int cur_id = new_intersection_instance.front();
							new_intersection_instance.pop();
							for (int j = 0; j < num; j++)
							{
								cv::Mat rect_intersection;

								if (intersection_flag[j] >= 1) continue;
								if (iter_class->second[indice[cur_id]]->frozen || iter_class->second[indice[j]]->frozen)
								{
									intersection_flag[j] = 0;
								}
								else
								{
									intersection_flag[j] = cv::rotatedRectangleIntersection(
										cv::RotatedRect(
											cv::Point2f(iter_class->second[indice[cur_id]]->mean_metric(0), iter_class->second[indice[cur_id]]->mean_metric(1)),
											cv::Size2f(iter_class->second[indice[cur_id]]->h(), iter_class->second[indice[cur_id]]->w()),
											atan2(iter_class->second[indice[cur_id]]->d()(1), iter_class->second[indice[cur_id]]->d()(0)) * R2D),
										cv::RotatedRect(
											cv::Point2f(iter_class->second[indice[j]]->mean_metric(0), iter_class->second[indice[j]]->mean_metric(1)),
											cv::Size2f(iter_class->second[indice[j]]->h(), iter_class->second[indice[j]]->w()),
											atan2(iter_class->second[indice[j]]->d()(1), iter_class->second[indice[j]]->d()(0)) * R2D)
										, rect_intersection);
								}

								if (intersection_flag[j] >= 1)
								{
									if (cluster_flag[indice[j]] != -1) // Bidirectinal matching!!!!
									{
										same_cluster_flag.push_back(cluster_flag[indice[j]]);
										continue;
									}
									new_intersection_instance.push(j);
								}

							}
						}
					}

					if (num >= 1)
					{
						for (int j = 0; j < num; j++)
						{
							if (intersection_flag[j] > 0)
								cluster_flag[indice[j]] = cluster_id;
						}
						cluster_id++;
					}
				}
			}

			for (int i = 0; i < iter_class->second.size(); i++)
			{
				if (cluster_flag[i] != -1)
				{
					(cluster_index[cluster_flag[i]]).push_back(i);
				}
			}

			for (int i = 0; i < cluster_id; i++)
			{
				if (cluster_index[i].size() == 0) continue; // This shouldn't happen. To be fixed.

				Eigen::Vector3d mean_ref = iter_class->second[cluster_index[i][0]]->mean_metric;

				// still active
				if ((mode == 0 && (Rwv.transpose() * (mean_ref - twv)).y() > -config.mapping_patch_freeze_distance && twv != Vector3d::Zero())
					|| (twv == Vector3d::Zero() && cluster_index[i].size() == 1 && iter_class->second[cluster_index[i][0]]->frozen == false))
				{
					for (int j = 0; j < cluster_index[i].size(); j++)
					{
						patches_new[iter_class->first].push_back(iter_class->second[cluster_index[i][j]]);
					}
					continue;
				}
				if (mode == 0 && cluster_index[i].size() == 1 && iter_class->second[cluster_index[i][0]]->frozen == false)
					continue;


				vector<double> eigen_value_list;
				for (int j = 0; j < cluster_index[i].size(); j++)
				{
					eigen_value_list.push_back(iter_class->second[cluster_index[i][j]]->h());
				}
				std::sort(eigen_value_list.begin(), eigen_value_list.end());
				//if (eigen_value_list.size() < 4) continue;
				double median_eigen_value = eigen_value_list[(eigen_value_list.size()) / 2];



				vector<Eigen::Vector4d> ds;
				vector<Eigen::Vector4d> uncs;
				vector<double> angles;

				for (int j = 0; j < cluster_index[i].size(); j++)
				{
					auto& patch = iter_class->second[cluster_index[i][j]];
					Vector3d ud0 = (mean_ref - patch->b_point_metric[0]).cross((patch->b_point_metric[1] - patch->b_point_metric[0]).normalized()); float d0 = ud0.norm(); if (ud0(2) > 0) d0 *= -1;
					Vector3d ud1 = (mean_ref - patch->b_point_metric[1]).cross((patch->b_point_metric[2] - patch->b_point_metric[1]).normalized()); float d1 = ud1.norm(); if (ud1(2) > 0) d1 *= -1;
					Vector3d ud2 = (mean_ref - patch->b_point_metric[2]).cross((patch->b_point_metric[3] - patch->b_point_metric[2]).normalized()); float d2 = ud2.norm(); if (ud2(2) > 0) d2 *= -1;
					Vector3d ud3 = (mean_ref - patch->b_point_metric[3]).cross((patch->b_point_metric[0] - patch->b_point_metric[3]).normalized()); float d3 = ud3.norm(); if (ud3(2) > 0) d3 *= -1;
					ds.push_back(Eigen::Vector4d(d0, d1, d2, d3));
					uncs.push_back(Eigen::Vector4d(patch->b_unc_dist[0], patch->b_unc_dist[1], patch->b_unc_dist[2], patch->b_unc_dist[3]));
					Vector3d uu = patch->b_point_metric[2] - patch->b_point_metric[1];
					angles.push_back(atan2(uu.y(), uu.x()));
				}

				Eigen::Vector4d ds_new;
				Eigen::Vector4d uncs_new;
				double angle_ref = angles[0];
				for (int ii = angles.size() - 1; ii >= 0; ii--)
				{
					angles[ii] = fmod(angles[ii] - angle_ref + M_PI * 3, M_PI * 2) - M_PI;
				}
				auto angles_temp = angles;
				std::sort(angles_temp.begin(), angles_temp.end());
				double angle_median = angle_ref + angles_temp[angles_temp.size() / 2];

				// Optimize the bounding box.
				for (int dd = 0; dd < 4; dd++)
				{
					double x = -99999;
					for (int ii = 0; ii < ds.size(); ii++)
					{
						if (x < ds[ii](dd)) x = ds[ii](dd);
					}

					double min_unc;
					double min_x = 0.0;
					for (int iiter = 0; iiter < 10; iiter++)
					{
						min_unc = 1000;
						double H = 0.0;
						double v = 0.0;
						for (int ii = 0; ii < ds.size(); ii++)
						{
							if (mode == 0 && fabs(angles[ii] + angle_ref - angle_median) > 3.0 / 180 * M_PI)
							{
								double zzz = ds_new.norm();
								continue;
							}
							if (uncs[ii](dd) > 1.0) continue;
							double l = (ds[ii](dd) - x) / uncs[ii](dd);
							double A = 1 / uncs[ii](dd);
							double downweight = sqrt(fabs(l));
							if (downweight > 1.5) continue;
							l /= 1;
							A /= 1;
							H += A * A;
							v += A * l;
							if (uncs[ii](dd) < min_unc)
							{
								min_x = ds[ii](dd);
								min_unc = uncs[ii](dd);
							}
						}
						x += v / H;
					}
					ds_new(dd) = min_x;
					uncs_new(dd) = min_unc;
				}
				Matrix3d direction_rotation;
				angle_median = -angle_median + M_PI / 2;
				direction_rotation << cos(angle_median), sin(angle_median), 0,
					-sin(angle_median), cos(angle_median), 0,
					0, 0, 1;
#ifdef DEBUG
				Eigen::Vector3d px0(-ds_new(3), -ds_new(0), 0);
				Eigen::Vector3d px1(ds_new(1), -ds_new(0), 0);
				Eigen::Vector3d px2(ds_new(1), ds_new(2), 0);
				Eigen::Vector3d px3(-ds_new(3), ds_new(2), 0);

				Vector3d b_point_metric[4];
				b_point_metric[0] = mean_ref + direction_rotation * px0;
				b_point_metric[1] = mean_ref + direction_rotation * px1;
				b_point_metric[2] = mean_ref + direction_rotation * px2;
				b_point_metric[3] = mean_ref + direction_rotation * px3;
				//patch_new.b_unc_dist[0] = uncs_new(0);
				//patch_new.b_unc_dist[1] = uncs_new(1);
				//patch_new.b_unc_dist[2] = uncs_new(2);
				//patch_new.b_unc_dist[3] = uncs_new(3);


				for (int j = 0; j < cluster_index[i].size(); j++)
				{
					auto& patch = iter_class->second[cluster_index[i][j]];
					Vector3d ud0 = (mean_ref - patch->b_point_metric[0]).cross((patch->b_point_metric[1] - patch->b_point_metric[0]).normalized()); float d0 = ud0.norm(); if (ud0(2) > 0) d0 *= -1;
					Vector3d ud1 = (mean_ref - patch->b_point_metric[1]).cross((patch->b_point_metric[2] - patch->b_point_metric[1]).normalized()); float d1 = ud1.norm(); if (ud1(2) > 0) d1 *= -1;
					Vector3d ud2 = (mean_ref - patch->b_point_metric[2]).cross((patch->b_point_metric[3] - patch->b_point_metric[2]).normalized()); float d2 = ud2.norm(); if (ud2(2) > 0) d2 *= -1;
					Vector3d ud3 = (mean_ref - patch->b_point_metric[3]).cross((patch->b_point_metric[0] - patch->b_point_metric[3]).normalized()); float d3 = ud3.norm(); if (ud3(2) > 0) d3 *= -1;

					float x0 = (patch->b_point_metric[0] - mean_ref).x() * 110 + 500;
					float y0 = (patch->b_point_metric[0] - mean_ref).y() * 110 + 500;
					float x1 = (patch->b_point_metric[1] - mean_ref).x() * 110 + 500;
					float y1 = (patch->b_point_metric[1] - mean_ref).y() * 110 + 500;
					float x2 = (patch->b_point_metric[2] - mean_ref).x() * 110 + 500;
					float y2 = (patch->b_point_metric[2] - mean_ref).y() * 110 + 500;
					float x3 = (patch->b_point_metric[3] - mean_ref).x() * 110 + 500;
					float y3 = (patch->b_point_metric[3] - mean_ref).y() * 110 + 500;
					cv::line(mm, cv::Point2f(x0, 1000 - y0), cv::Point2f(x1, 1000 - y1), cv::Scalar(255, 255, 255));
					cv::line(mm, cv::Point2f(x1, 1000 - y1), cv::Point2f(x2, 1000 - y2), cv::Scalar(255, 255, 255));
					cv::line(mm, cv::Point2f(x2, 1000 - y2), cv::Point2f(x3, 1000 - y3), cv::Scalar(255, 255, 255));
					//cv::line(mm, cv::Point2f(x3, 1000-y3), cv::Point2f(x0, 1000-y0), cv::Scalar(255, 255, 255));

					cv::Point2f p0 = cv::Point2f((x0 + x1) / 2, (y0 + y1) / 2);
					cv::Point2f p1 = cv::Point2f((x1 + x2) / 2, (y1 + y2) / 2);
					cv::Point2f p2 = cv::Point2f((x2 + x3) / 2, (y2 + y3) / 2);
					cv::Point2f p3 = cv::Point2f((x3 + x0) / 2, (y3 + y0) / 2);
					cv::circle(mm, cv::Point2f(p0.x, 1000 - p0.y), 5, cv::Scalar(255, 255, 255));
					cv::circle(mm, cv::Point2f(p1.x, 1000 - p1.y), 5, cv::Scalar(255, 255, 255));
					cv::circle(mm, cv::Point2f(p2.x, 1000 - p2.y), 5, cv::Scalar(255, 255, 255));
					cv::circle(mm, cv::Point2f(p3.x, 1000 - p3.y), 5, cv::Scalar(255, 255, 255));
					Eigen::Vector3d u0 = (patch->b_point_metric[1] - patch->b_point_metric[0]).normalized();
					Eigen::Vector3d u1 = (patch->b_point_metric[2] - patch->b_point_metric[1]).normalized();

					cv::Point2f pp0, pp1;
					pp0 = p0 - cv::Point2f(u1.x(), u1.y()) * patch->b_unc_dist[0] * 110; pp1 = p0 + cv::Point2f(u1.x(), u1.y()) * patch->b_unc_dist[0] * 110;
					cv::line(mm, cv::Point2f(pp0.x, 1000 - pp0.y), cv::Point2f(pp1.x, 1000 - pp1.y), cv::Scalar(0, 0, 255));
					pp0 = p1 - cv::Point2f(u0.x(), u0.y()) * patch->b_unc_dist[1] * 110; pp1 = p1 + cv::Point2f(u0.x(), u0.y()) * patch->b_unc_dist[1] * 110;
					cv::line(mm, cv::Point2f(pp0.x, 1000 - pp0.y), cv::Point2f(pp1.x, 1000 - pp1.y), cv::Scalar(0, 0, 255));
					pp0 = p2 - cv::Point2f(u1.x(), u1.y()) * patch->b_unc_dist[2] * 110; pp1 = p2 + cv::Point2f(u1.x(), u1.y()) * patch->b_unc_dist[2] * 110;
					cv::line(mm, cv::Point2f(pp0.x, 1000 - pp0.y), cv::Point2f(pp1.x, 1000 - pp1.y), cv::Scalar(0, 0, 255));
					pp0 = p3 - cv::Point2f(u0.x(), u0.y()) * patch->b_unc_dist[3] * 110; pp1 = p3 + cv::Point2f(u0.x(), u0.y()) * patch->b_unc_dist[3] * 110;
					cv::line(mm, cv::Point2f(pp0.x, 1000 - pp0.y), cv::Point2f(pp1.x, 1000 - pp1.y), cv::Scalar(0, 0, 255));
					std::cout << patch->b_unc_dist[0] << " "
						<< patch->b_unc_dist[1] << " "
						<< patch->b_unc_dist[2] << " "
						<< patch->b_unc_dist[3] << std::endl;
				}



				float x0 = (b_point_metric[0] - mean_ref).x() * 110 + 500;
				float y0 = (b_point_metric[0] - mean_ref).y() * 110 + 500;
				float x1 = (b_point_metric[1] - mean_ref).x() * 110 + 500;
				float y1 = (b_point_metric[1] - mean_ref).y() * 110 + 500;
				float x2 = (b_point_metric[2] - mean_ref).x() * 110 + 500;
				float y2 = (b_point_metric[2] - mean_ref).y() * 110 + 500;
				float x3 = (b_point_metric[3] - mean_ref).x() * 110 + 500;
				float y3 = (b_point_metric[3] - mean_ref).y() * 110 + 500;
				cv::line(mm, cv::Point2f(x0, 1000 - y0), cv::Point2f(x1, 1000 - y1), cv::Scalar(0, 255, 0));
				cv::line(mm, cv::Point2f(x1, 1000 - y1), cv::Point2f(x2, 1000 - y2), cv::Scalar(0, 255, 0));
				cv::line(mm, cv::Point2f(x2, 1000 - y2), cv::Point2f(x3, 1000 - y3), cv::Scalar(0, 255, 0));
				//cv::line(mm, cv::Point2f(x3, 1000-y3), cv::Point2f(x0, 1000-y0), cv::Scalar(255, 255, 255));

				cv::Point2f p0 = cv::Point2f((x0 + x1) / 2, (y0 + y1) / 2);
				cv::Point2f p1 = cv::Point2f((x1 + x2) / 2, (y1 + y2) / 2);
				cv::Point2f p2 = cv::Point2f((x2 + x3) / 2, (y2 + y3) / 2);
				cv::Point2f p3 = cv::Point2f((x3 + x0) / 2, (y3 + y0) / 2);
				cv::circle(mm, cv::Point2f(p0.x, 1000 - p0.y), 5, cv::Scalar(0, 255, 0));
				cv::circle(mm, cv::Point2f(p1.x, 1000 - p1.y), 5, cv::Scalar(0, 255, 0));
				cv::circle(mm, cv::Point2f(p2.x, 1000 - p2.y), 5, cv::Scalar(0, 255, 0));
				cv::circle(mm, cv::Point2f(p3.x, 1000 - p3.y), 5, cv::Scalar(0, 255, 0));
				cv::circle(mm, cv::Point2f(500, 500), 15, cv::Scalar(255, 255, 0));
				Eigen::Vector3d u0 = (b_point_metric[1] - b_point_metric[0]).normalized();
				Eigen::Vector3d u1 = (b_point_metric[2] - b_point_metric[1]).normalized();

				cv::imshow("mm_temp", mm);
				cv::waitKey(1);
#endif

				RoadInstancePatch patch_new;
				double best_score = 10000;
				int best_score_id = -1;

				for (int j = 0; j < cluster_index[i].size(); j++)
				{
					if (iter_class->second[cluster_index[i][j]]->percept_distance < best_score
						&& iter_class->second[cluster_index[i][j]]->h() >= median_eigen_value - 0.1)
					{
						best_score = iter_class->second[cluster_index[i][j]]->percept_distance;
						best_score_id = j;
					}
				}
				if (((ds_new(3) + ds_new(1)) * (ds_new(2) + ds_new(0)) < 2.0 || (ds_new(2) + ds_new(0)) < 0.5) && iter_class->first == PatchType::GUIDE)
					continue;

				patch_new = *iter_class->second[cluster_index[i][best_score_id]];
				Eigen::Vector3d p0(-ds_new(3), -ds_new(0), 0);
				Eigen::Vector3d p1(ds_new(1), -ds_new(0), 0);
				Eigen::Vector3d p2(ds_new(1), ds_new(2), 0);
				Eigen::Vector3d p3(-ds_new(3), ds_new(2), 0);
				patch_new.b_point_metric[0] = mean_ref + direction_rotation * p0;
				patch_new.b_point_metric[1] = mean_ref + direction_rotation * p1;
				patch_new.b_point_metric[2] = mean_ref + direction_rotation * p2;
				patch_new.b_point_metric[3] = mean_ref + direction_rotation * p3;
				patch_new.b_unc_dist[0] = uncs_new(0);
				patch_new.b_unc_dist[1] = uncs_new(1);
				patch_new.b_unc_dist[2] = uncs_new(2);
				patch_new.b_unc_dist[3] = uncs_new(3);
				patch_new.merged = true;
				patch_new.frozen = true;


				patches_new[iter_class->first].push_back(make_shared<RoadInstancePatch>(patch_new));
			}
		}
		else if (iter_class->first == PatchType::SOLID || iter_class->first == PatchType::STOP)
		{
			auto opt_t0 = chrono::system_clock::now();

			for (int i = 0; i < iter_class->second.size(); i++)
			{
				if (cluster_flag[i] == -1)
				{
					vector<int> indice(1000);
					vector<float> dist(1000);
					int num = pts.size() > 10 ? 10 : pts.size();
					kdtree.knnSearch(vector<float>({ pts[i].x,pts[i].y }), indice, dist, num);

					vector<int> intersection_flag(num, -1);
					queue<int> new_intersection_instance; new_intersection_instance.push(0);
					//vector<pair<Matrix3d, Vector3d>> pts_vis;

					vector<int> same_cluster_flag; // for bi-directional mathching

					if (iter_class->second[i]->frozen)
					{
						intersection_flag[0] = 1;
					}
					else
					{
						// Clustering the line segments.
						while (new_intersection_instance.size() > 0)
						{
							int cur_id = new_intersection_instance.front();
							new_intersection_instance.pop();
							for (int j = 0; j < num; j++)
							{
								cv::Mat rect_intersection;

								if (intersection_flag[j] >= 1) continue;

								auto d1 = iter_class->second[indice[cur_id]]->d();
								auto d2 = iter_class->second[indice[j]]->d();
								double costheta = d1.dot(d2);


								if (fabs(costheta) < cos(90.0 / 180 * M_PI))
								{
									intersection_flag[j] = 0;
								}
								else if ((iter_class->second[indice[cur_id]]->frozen || iter_class->second[indice[j]]->frozen) && (indice[cur_id] != indice[j]))
								{
									intersection_flag[j] = 0;
								}
								else
								{
									double min_dist = 1e9;
									double min_dist_across = 1e9;
									double min_dist_across2 = 1e9;
									double min_costheta = 1e9;
									double this_dist, this_dist_across, this_dist_across2, this_costheta;
									for (int m = 0; m < iter_class->second[indice[cur_id]]->line_points_metric.size(); m++)
										for (int n = 0; n < iter_class->second[indice[j]]->line_points_metric.size(); n++)
										{
											Vector3d direction_m_local;
											if (m == 0)  direction_m_local = (iter_class->second[indice[cur_id]]->line_points_metric[m + 1] - iter_class->second[indice[cur_id]]->line_points_metric[m]).normalized();
											else         direction_m_local = (iter_class->second[indice[cur_id]]->line_points_metric[m] - iter_class->second[indice[cur_id]]->line_points_metric[m - 1]).normalized();
											Vector3d direction_n_local;
											if (n == 0)  direction_n_local = (iter_class->second[indice[j]]->line_points_metric[n + 1] - iter_class->second[indice[j]]->line_points_metric[n]).normalized();
											else         direction_n_local = (iter_class->second[indice[j]]->line_points_metric[n] - iter_class->second[indice[j]]->line_points_metric[n - 1]).normalized();

											this_dist = (iter_class->second[indice[cur_id]]->line_points_metric[m] - iter_class->second[indice[j]]->line_points_metric[n]).norm();
											this_dist_across = fabs((iter_class->second[indice[cur_id]]->line_points_metric[m] - iter_class->second[indice[j]]->line_points_metric[n]).dot(
												Vector3d(-direction_m_local(1), direction_m_local(0), direction_m_local(2))));
											this_dist_across2 = fabs((iter_class->second[indice[cur_id]]->line_points_metric[m] - iter_class->second[indice[j]]->line_points_metric[n]).dot(
												Vector3d(-direction_n_local(1), direction_n_local(0), direction_n_local(2))));
											this_costheta = fabs(direction_m_local.dot(direction_n_local));
											if (this_dist < min_dist)
											{
												min_dist = this_dist;
												min_dist_across = this_dist_across;
												min_dist_across2 = this_dist_across2;
												min_costheta = this_costheta;
											}
											comp_count++;
										}
									if (((min_dist_across < config.mapping_line_cluster_max_across_dist2 && min_dist_across2 < config.mapping_line_cluster_max_across_dist1) ||
										(min_dist_across < config.mapping_line_cluster_max_across_dist1 && min_dist_across2 < config.mapping_line_cluster_max_across_dist2))
										&& min_dist < config.mapping_line_cluster_max_dist && min_costheta > cos(config.mapping_line_cluster_max_theta / 180 * M_PI)) intersection_flag[j] = 1;

								}
								if (intersection_flag[j] >= 1)
								{
									if (cluster_flag[indice[j]] != -1) // Bidirectinal matching!!!!
									{
										same_cluster_flag.push_back(cluster_flag[indice[j]]);
										continue;
									}
									new_intersection_instance.push(j);
								}
							}
						}
					}

					if (num >= 1)
					{
						for (int j = 0; j < num; j++)
						{
							if (intersection_flag[j] > 0)
							{
								cluster_flag[indice[j]] = cluster_id;
							}
						}

						if (same_cluster_flag.size() > 0)
						{
							sort(same_cluster_flag.begin(), same_cluster_flag.end());
							for (int j = 0; j < num; j++)
								if (intersection_flag[j] > 0)
									cluster_flag[indice[j]] = same_cluster_flag[0];

							for (int iii = 0; iii < iter_class->second.size(); iii++)
								for (int iiii = 0; iiii < same_cluster_flag.size(); iiii++)
									if (cluster_flag[iii] == same_cluster_flag[iiii]) cluster_flag[iii] = same_cluster_flag[0];
						}
						cluster_id++;
					}
				}
			}
			auto opt_t1 = chrono::system_clock::now();
			for (int i = 0; i < iter_class->second.size(); i++)
			{
				if (cluster_flag[i] != -1)
				{
					(cluster_index[cluster_flag[i]]).push_back(i);
				}
			}
			auto end1 = chrono::system_clock::now();

			for (int i = 0; i < cluster_id; i++)
			{
				vector<shared_ptr<RoadInstancePatch>> all_patches_this_cluster;
				for (int j = 0; j < cluster_index[i].size(); j++)
				{
					all_patches_this_cluster.push_back(iter_class->second[cluster_index[i][j]]);
				}
				if (all_patches_this_cluster.size() < 1) continue;
				shared_ptr<RoadInstancePatch> line_est;

				auto ret = LineCluster2SingleLine(iter_class->first, all_patches_this_cluster, line_est, Rwv);

				if (ret < 0) continue;


				patches_new[iter_class->first].push_back(line_est);


			}
		}
		auto end = chrono::system_clock::now();
	}
	patches = patches_new;



	//** Refreshing linked frames
	// This is used for geo-registering.
	for (auto iter_class = patches.begin(); iter_class != patches.end(); iter_class++)
	{
		for (int i = 0; i < iter_class->second.size(); i++)
		{
			auto& patch = iter_class->second[i];

			if (patch->frozen) continue;

			if (iter_class->first == PatchType::DASHED || iter_class->first == PatchType::GUIDE)
			{
				patch->linked_frames.clear();
				patch->linked_frames.resize(1);
				for (auto iter_pose = queued_poses.lower_bound(queued_poses.rbegin()->first - 200); iter_pose != queued_poses.end(); iter_pose++)
				{
					Eigen::Vector3d tvf = iter_pose->second.first.transpose() *
						(patch->mean_metric - iter_pose->second.second);
					if (tvf.x() < 10 && tvf.x() > -10 && tvf.y() > 2 && tvf.y() < 20) // in the region
					{
						patch->linked_frames[0].push_back(iter_pose->first);
					}
				}
			}
			else
			{
				patch->linked_frames.clear();
				for (int jjj = 0; jjj < patch->line_points_metric.size(); jjj++)
				{
					patch->linked_frames.push_back(vector<long long>());
					for (auto iter_pose = queued_poses.lower_bound(queued_poses.rbegin()->first - 200); iter_pose != queued_poses.end(); iter_pose++)
					{
						Eigen::Vector3d tvf = iter_pose->second.first.transpose() *
							(patch->line_points_metric[jjj] - iter_pose->second.second);
						if (tvf.x() < 10 && tvf.x() > -10 && tvf.y() > 2 && tvf.y() < 20) // in the region
						{
							patch->linked_frames[jjj].push_back(iter_pose->first);
						}
					}
				}

			}
		}
	}

	// Attention!!!
	// We simply freeze old lines to avoid troublesome cases.
	// This could be optimized.
	if (mode == 0)
		for (auto iter_class = patches.begin(); iter_class != patches.end(); iter_class++)
		{
			for (int i = 0; i < iter_class->second.size(); i++)
			{
				auto& patch = iter_class->second[i];
				if (iter_class->first == PatchType::SOLID || iter_class->first == PatchType::STOP)
					if ((Rwv.transpose() * (patch->line_points_metric.back() - twv)).y() < -config.mapping_line_freeze_distance ||
						patch->line_points_metric.size() > config.mapping_line_freeze_max_length || twv == Vector3d::Zero())
						patch->frozen = true;
				if (iter_class->first == PatchType::DASHED || iter_class->first == PatchType::GUIDE)
					if ((Rwv.transpose() * (patch->mean_metric - twv)).y() < -config.mapping_line_freeze_distance)
						patch->frozen = true;
			}
		}
	return 0;
}