#ifndef GVIEWER_H
#define GVIEWER_H
#include <Eigen/Dense>
#include <GLFW/glfw3.h>
#include <vector>
#include <utility>
#include <thread>
#include <mutex>
#include <math.h>
#include <map>
#include <opencv2/opencv.hpp>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

using namespace std;
typedef vector<pair<Eigen::Matrix3d, Eigen::Vector3d>> Frames;
typedef vector<Eigen::Vector3d> VPointCloud;

enum VisualizedPatchType { BOX=0,LINE_SEGMENT =1 };

struct VisualizedInstance
{
	VisualizedPatchType type;
	double color[3];
	Eigen::Matrix3d R;
	Eigen::Vector3d t;
	double w, l, h;
	vector<Eigen::Vector3d> pts;
	vector<Eigen::Vector3d> pts_color;
	float alpha = 0.75f;
	float linewidth = 3.0f;
};


class gviewer
{
public:
	gviewer();
	~gviewer();
public:
	void Show();
	void ClearView();
	void SetFrames(const Frames& t);
	void SetFrames(const vector<Frames>& vt);
	void AddNewFrame(const pair<Eigen::Matrix3d, Eigen::Vector3d>& f);

	void SetPointCloud(const VPointCloud& pc);
	void SetPointCloud(const vector<VPointCloud>& vpc);
	void SetPointCloudSemantic(const vector<VPointCloud>& vpc);
	void SetInstances(const vector<VisualizedInstance>& vi);
	void AddNewPoint(const Eigen::Vector3d& p);
	void AddNewPoint(const vector<Eigen::Vector3d>& p);

	void SetTrajectory(const VPointCloud& t);
	void SetTrajectory(const vector<VPointCloud>& vt);
	void AddNewPos(const Eigen::Vector3d& p);
	void ScreenShot(const string& s);
	void SetCenter();
	void Hide();
private:
	void Run();

private:
	static vector<gviewer*> vptr;
	GLFWwindow* window;
	thread* t;
	mutex m_mutex;
	vector<VPointCloud> mv_trajectory;
	vector<VPointCloud> mv_pointCloud;
	vector<Frames> mv_frames;
	vector<VPointCloud> mv_pointCloud_semantic;
	vector<VisualizedInstance> mv_instances;
	bool need_screenshot = false;
	bool set_center = false;
	string screenshot_path = "";
};
#endif
