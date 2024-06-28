
#include "roadlib.h"
#include "gviewer.h"
#include "visualization.h"
#include <fstream>


gviewer viewer;
vector<VisualizedInstance> vis_instances;
std::normal_distribution<double> noise_distribution;
std::default_random_engine random_engine; 

// Map-aided localization.
int main_phase_localization(const SensorConfig& config,
	const map<double, string>& camstamp,
	const string& raw_dir, const string& semantic_dir,
	RoadInstancePatchMap& road_map,
	const Trajectory& traj_vio_user,
	const pair<double,Tpoint>& initial_guess,
	Trajectory& traj_user, string result_file);

int main(int argc, char *argv[])
{
	if(argc!=9)
	{
		std::cerr<<"[ERROR] Number of arguments not correct!"<<std::endl;
		std::cerr<<"[USAGE] demo_mapping CONFIG_PATH IMAGE_TIMESTAMP_FILE IMAGE_DIR SEMANTIC_DIR REF_POSE_FILE ODO_POSE_FILE MAP_FILE RESULT_POSE_FILE"<<std::endl;
		exit(1);
	}
	string config_file(argv[1]);// = "D:/Projects/road_vision_iros/config/0412/vi.yaml";
	string stamp_file(argv[2]); //= "D:/city_0412/image/stamp_rearrange.txt";
	string raw_dir(argv[3]); //= "D:/city_0412/image/cam0";
	string semantic_dir(argv[4]); //= "D:/city_0412/image/out/pred";
	string ref_file(argv[5]); //= "D:/city_0412/gt.txt"
	string odo_file(argv[6]); //= "D:/city_0412/odo.txt";
	string map_file(argv[7]); //= "D:/city_0412/map.bin";
	string result_file(argv[8]); //= "D:/city_0412/result.txt";

	viewer.Show();

	SensorConfig config (config_file);

	Trajectory traj_gt = load_global_trajectory(ref_file); // Only used for initial guess.
	Trajectory traj_odo = load_local_trajectory(odo_file);
	map<double, string> camstamp = load_camstamp(stamp_file);

	// Load map.
	RoadInstancePatchMap road_map;
	road_map.loadMapFromFileBinaryRaw(map_file);
	road_map.unfreeze();
	
	// In this example, initial guess of the pose is needed.
    traj_gt.transfrom_ref(road_map.ref);
	pair<double, Tpoint> initial_guess = make_pair(config.t_start, traj_gt.poses[config.t_start]);
	std::cout<<"[INFO] initial guess: "<<setprecision(3)<<setiosflags(ios::fixed)<<config.t_start<<" "
		<<initial_guess.second.t.transpose()<<std::endl;
	
	// Map-aided localization.
	Trajectory traj_result;

	main_phase_localization(config, camstamp, raw_dir, semantic_dir,
		road_map, traj_odo, initial_guess, traj_result, result_file);

	return 0;
}
