
#include "roadlib.h"
#include "gviewer.h"
#include "visualization.h"
#include <fstream>
//using std::experimental::filesystem::v1::directory_iterator;
//using std::experimental::filesystem::v1::path;


gviewer viewer;
vector<VisualizedInstance> vis_instances;
std::normal_distribution<double> noise_distribution;
std::default_random_engine random_engine; 

//// Posegraph optimization
//int main_phase1(const map<double, TrajectoryPoint> &traj_vio,
//	const map<double, TrajectoryPoint> &traj_gnss,
//	map<double, TrajectoryPoint> &traj_ref, double t_start, double t_end);

// Road map generating
int main_phase_mapping(const SensorConfig & config, 
	map<double, string> camstamp, string raw_dir, string semantic_dir,
	const Trajectory & traj_ref,
	map<double, shared_ptr<RoadInstancePatchFrame>> & all_frames,
	RoadInstancePatchMap &road_map);

int main(int argc, char *argv[])
{
	if(argc!=8)
	{
		std::cerr<<"[ERROR] Number of arguments not correct!"<<std::endl;
		std::cerr<<"[USAGE] demo_mapping CONFIG_PATH IMAGE_TIMESTAMP_FILE IMAGE_DIR SEMANTIC_DIR REF_POSE_FILE ODO_POSE_FILE RESULT_MAP_FILE"<<std::endl;
		exit(1);
	}
	string config_file(argv[1]);// = "D:/Projects/road_vision_iros/config/0412/vi.yaml";
	string stamp_file(argv[2]); //= "D:/city_0412/image/stamp_rearrange.txt";
	string raw_dir(argv[3]); //= "D:/city_0412/image/cam0";
	string semantic_dir(argv[4]); //= "D:/city_0412/image/out/pred";
	string ref_file(argv[5]); //= "D:/city_0412/gt.txt"
	string odo_file(argv[6]); //= "D:/city_0412/odo.txt";
	string result_file(argv[7]); //= "D:/city_0412/odo.txt";

	viewer.Show();
	RoadInstancePatchMap road_map;
	map<double, shared_ptr<RoadInstancePatchFrame>> all_frames;

	SensorConfig config (config_file);

	Trajectory traj_gt = load_global_trajectory(ref_file);
	Trajectory traj_odo = load_local_trajectory(odo_file);

	map<double, string> camstamp = load_camstamp(stamp_file);

	// Incremental mapping.
	main_phase_mapping(config, camstamp, raw_dir, semantic_dir, traj_odo, all_frames, road_map);

	// Incremental mapping.
	road_map.geoRegister(traj_gt, vis_instances);
	road_map.cleanMap();
	road_map.mergePatches(config, 1);
	road_map.saveMapToFileBinaryRaw(result_file);
	
	//road_map.loadMapFromFileBinaryRaw("D:/city_0412/map0.bin");
	//visualize_roadmap(road_map, vis_instances);
	//viewer.SetInstances(vis_instances);
	//cin.get();

	while (true)
	{

	}

}
