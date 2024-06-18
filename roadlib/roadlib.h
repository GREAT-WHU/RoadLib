#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include<Eigen/StdVector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/flann.hpp>

#include <iostream>
#include <filesystem>
#include <chrono>
#include <unordered_map>
#include <iomanip>
#include <set>
#include <random>
#include "gv_utils.h"
#include "ipm_processer.h"
#include "utils.hpp"
#include "gviewer.h"

using namespace Eigen;
using namespace std;

struct SensorConfig
{
public:
	SensorConfig(string path);
	SensorConfig() {};
public:
	gv::CameraConfig cam;
	
	int pose_smooth_window = 20;
	bool need_smooth = true;
	double large_slope_thresold = 1.5;
	double t_start;
	double t_end;

	int patch_min_size = 50;
	double patch_dashed_min_h = 1.35;
	double patch_dashed_max_h = 10.0;
	double patch_dashed_max_dist = 12.0;
	double patch_guide_min_h = 0.0;
	double patch_guide_max_h = 1000.0;
	double patch_guide_max_dist = 20.0;
	double patch_solid_max_dist = 15.0;
	double patch_stop_max_dist = 12.0;

	int mapping_step = 10;
	double mapping_patch_freeze_distance = 10.0;
	double mapping_line_freeze_distance = 10.0;
	double mapping_line_freeze_max_length = 50.0;
	double mapping_line_cluster_max_dist = 1.0;
	double mapping_line_cluster_max_across_dist1 = 1.0;
	double mapping_line_cluster_max_across_dist2 = 0.4;
	double mapping_line_cluster_max_theta = 10.0;

	bool enable_vis_image = true;
	bool enable_vis_3d = true;
};

extern gviewer viewer;
extern vector<VisualizedInstance> vis_instances;
extern std::normal_distribution<double> noise_distribution;
extern std::default_random_engine random_engine;

enum PatchType { EMPTY = -1, SOLID = 0, DASHED = 1, GUIDE = 2, ZEBRA = 3, STOP = 4 };

inline PatchType gray2class(int gray)
{
	if (gray == 2) return PatchType::DASHED;
	else if (gray == 3) return PatchType::GUIDE;
	else if (gray == 4) return PatchType::ZEBRA;
	else if (gray == 5) return PatchType::STOP;
	else if (gray > 0) return PatchType::SOLID;
	else return PatchType::EMPTY;
}

inline string PatchType2str(PatchType PatchType)
{
	if (PatchType == PatchType::DASHED) return "dashed";
	else if (PatchType == PatchType::GUIDE) return "guide";
	else if (PatchType == PatchType::ZEBRA) return "zebra";
	else if (PatchType == PatchType::SOLID) return "solid";
	else if (PatchType == PatchType::STOP) return "stop";
	else if (PatchType == PatchType::EMPTY) return "empty";
	else return "unknown";
}


class RoadInstancePatch
{
public:
	static long long next_id;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
	//** ID/Class
	long long id;
	PatchType road_class;
	map<PatchType, int> road_class_count;

	//** Flags
	bool line_valid;
	bool merged = false;
	bool frozen = false;
	bool valid_add_to_map = false;
	bool out_range = false;

	long long frame_id;

	// Associated with RoadInstancePatchMap::timestamps and queued_poses.
	// For patch-like instances -> linked_frames[0]
	// For line-like instances  -> linked_frames[i], i = 0, ..., line_points_metric.size()
	vector<vector<long long>> linked_frames; 

	//** Bounding box related
	// Main parameters for patch-like instances.
	// 
	//    p3----p2
	//    |      |
	//    |      |
	//    p0 --- p1
	//
	Eigen::Vector3d b_point[4];        // Image frame.
	Eigen::Vector3d b_point_metric[4]; // Body/map frame.
	double b_unc_dist[4];              // Distance uncertainty.

	//** Line related
	// Main parameters for line-like instances.
	Eigen::VectorXd line_koef;
	vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> line_points;
	vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> line_points_metric;
	vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> line_points_uncertainty;

	//** Raw points related (image frame and metric frame)
	// General attributes.
	// 1) Image frame
	double top, left, width, height;
	Eigen::Vector3d mean;
	Eigen::Matrix3d cov;
	Eigen::Vector3d direction;
	double eigen_value[3];
	vector<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points;

	// 2) Body/map frame
	Eigen::Vector3d mean_metric;
	vector<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points_metric;
	Eigen::Matrix3d mean_uncertainty;
	double percept_distance = 10000;


public:
	RoadInstancePatch()
	{
		id = next_id++;
		for (int ii = 0; ii < 4; ii++)
		{
			b_point[ii].setZero();
			b_point_metric[ii].setZero();
		}
	}

	// Height of bounding box.
	double h() const;

	// Width of bounding box.
	double w() const;

	// Direction of bounding box/line.
	Eigen::Vector3d d() const;

};

class RoadInstancePatchFrame
{
public:
	static long long next_id;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
	long long id;
	double time;
	Eigen::Matrix3d R; // R^world_body
	Eigen::Vector3d t; // t^world_body
	map<PatchType, vector<shared_ptr<RoadInstancePatch>>> patches;
public:
	RoadInstancePatchFrame()
	{
		id = next_id++;
	}

	// Calculate metric-scale properties of the patches.
	// Image frame -> body frame.
	int generateMetricPatches(const SensorConfig &config, const gv::IPMProcesser &ipm);
};

class RoadInstancePatchMap
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	map<PatchType, vector<shared_ptr<RoadInstancePatch>>> patches;
	Eigen::Vector3d ref;

	map<long long, pair<Matrix3d, Vector3d>> queued_poses;
	map<long long, double> timestamps;
public:
	int addFrame(const RoadInstancePatchFrame & frame);

	// Merge patches in the current map.
	// mode 0 : incremental mapping; mode 1: map merging/map checking
	// For mode 0, Rwv and twv are used to determine active instances and freeze old instances.
	// For mode 1, Rwv and twv are useless. Just use dummy values. (To be improved
	int mergePatches(const SensorConfig &config, const int mode, const Eigen::Matrix3d &Rwv = Eigen::Matrix3d::Identity(), const Eigen::Vector3d & twv = Eigen::Vector3d::Zero());
	
	// Simply stacking the patches from two maps.
	// The patches wouldn't be merged.
	int mergeMap(const RoadInstancePatchMap& road_map_other);

	// Clear the map.
	int clearMap();

	// Unfreeze the pathes (for further merging).
	int unfreeze();
	
	// Integrity checking.
	int cleanMap();

	// Save/load functions.
	// Notice that only metric-scale properties are saved.
	int saveMapToFileBinaryRaw(string filename);
	int loadMapFromFileBinaryRaw(string filename);

	// Build KDtree for map matching.
	int buildKDTree();

	// Instance-level nearest matching.
	map<PatchType, vector<pair<int, int>>> mapMatch(RoadInstancePatchFrame &frame,int mode = 0); // mode : 0(normal), 1(strict)
	
	// Line segment matching (for measurement construction).
	vector<pair<int, int>> getLineMatch(RoadInstancePatchFrame &frame, PatchType road_class,
		int frame_line_count, int map_line_count, int mode =0);

	// Geo-register the map elements based on linked frames.
	// Function 'mergePatches' should be called later for consistency.
	int geoRegister(const Trajectory& new_traj,
		vector<VisualizedInstance>& lines_vis);

public:
	map<long long, double> ignore_frame_ids; // frame_id - distance threshold
private:

};


/**
 * @brief					Merge the line instance cluster.
 * @param lines_in			The raw line cluser.
 * @param line_est			The merged line instance.
 *
 * @return					Success flag.
 */
extern int LineCluster2SingleLine(const PatchType road_class, const vector<shared_ptr<RoadInstancePatch>>& lines_in, shared_ptr<RoadInstancePatch>& line_est, Eigen::Matrix3d Rwv = Eigen::Matrix3d::Identity());

/**
 * @brief					Use the semantic IPM image to generate the road marking instances.
 * @param config			The raw line cluser.
 * @param ipm				IPM processor with up-to-date camera-ground parameters.
 * @param ipm_raw			RGB IPM image.
 * @param ipm_class			Semantic IPM image. (label = 0,1,2,3,4,5, other)
 *
 * @return					Success flag.
 */
extern RoadInstancePatchFrame generateInstancePatch(const SensorConfig& config, const gv::IPMProcesser& ipm, const cv::Mat& ipm_raw, const cv::Mat& ipm_class);

/**
 * @brief					Calculate the uncertainty of the element on the IPM image based on the pixel coordinates (uI, vI).
 * @param uI, vI			Pixel coordinates on the IPM image.
 *
 * @return					Uncertainty matrix.
 */
extern Matrix3d calcUncertainty(const gv::IPMProcesser& ipm, double uI, double vI);

extern int PointCloud2Curve2D(const vector<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& points, int dim, VectorXd& K);

extern int LabelClustering(const cv::Mat& ipm_class, cv::Mat& ipm_label, cv::Mat& stats, cv::Mat& centroids);