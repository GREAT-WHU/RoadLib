#include "gviewer.h"
#include <iostream>

void MyPerspective(GLdouble fov, GLdouble aspectRatio, GLdouble zNear, GLdouble zFar);
void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);
static void cursor_position_callback(GLFWwindow *window, double xpos, double ypos);
// map<GLFWwindow*, Viewer*> Viewer::mapptr;
// Viewer* getViewerPtr(GLFWwindow* window);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);
void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);
// Different Objects
void MyCylinder(GLdouble r, GLdouble l, int edgenum);
void MyFrame();
void MyAerial(GLfloat size);
void MyAxis(GLfloat size);
void MyBox(GLfloat l, GLfloat w, GLfloat h);
void MyAxisSimple(GLfloat scale);
void genTextListMap();
void showBottomText(string s);
void showStatusText(string s);

vector<gviewer *> gviewer::vptr;

using namespace std;

bool mb_OK;
bool mb_MPs = true;
bool mb_trajetory = true;
bool mb_KFs = true;
bool mb_overlook = false;
bool mb_track = true;
bool mb_grid = true;
bool mb_axis = true;
bool mb_instances = true;
double m_interval = 0.1;
double m_dist = 5;
double m_scale = 1;
double m_rot_X = 0;
double m_rot_Y = 0;
double m_rot_Z = 0;
double m_rot_X0 = 0;
double m_rot_Y0 = 0;
double m_rot_Z0 = 0;
double m_trans_X = 0;
double m_trans_Y = 0;
double m_trans_X0 = 0;
double m_trans_Y0 = 0;

double m_mouseL_x = 0;
double m_mouseL_y = 0;
double m_mouseR_x = 0;
double m_mouseR_y = 0;
double m_dx = 0;
double m_dy = 0;

double m_curr_X = 0;
double m_curr_Y = 0;
double m_curr_Z = 0;

double m_curr_X0 = 0;
double m_curr_Y0 = 0;
double m_curr_Z0 = 0;

bool is_pressL = false;
bool is_pressR = false;
bool mb_stop = false;

int m_width = 0;
int m_height = 0;

map<char, int> m_text_call_map;

gviewer::gviewer()
{
	t = nullptr;
	gviewer::vptr.push_back(this);

	mv_trajectory.resize(1);
	mv_frames.resize(1);
	mv_pointCloud.resize(1);
}

gviewer::~gviewer()
{
	gviewer::vptr.erase(std::find(gviewer::vptr.begin(), gviewer::vptr.end() - 1, this));
}

// Viewer* getViewerPtr(GLFWwindow* window)
//{
//	for (auto ite = Viewer::vptr.begin(); ite != Viewer::vptr.end(); ite++)
//	{
//		if ((*ite)->window == window)
//			return *ite;
//	}
//	return nullptr;
// }

void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
	glViewport(0, 0, width, height);
	m_width = width;
	m_height = height;
}
void error_callback(int error, const char *msg)
{
	std::string s;
	s = " [" + std::to_string(error) + "] " + msg + '\n';
	std::cerr << s << std::endl;
}

void gviewer::Run()
{
	/* Initialize the library */
	if (!glfwInit())
		return;
	glfwSetErrorCallback(error_callback);
	/* Create a windowed mode window and its OpenGL context */
	window = glfwCreateWindow(100, 100, "Viewer", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		return;
	}
	/* Make the window's context current */
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetCursorPosCallback(window, cursor_position_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetKeyCallback(window, key_callback);
	glfwSetWindowSize(window, 800, 600);
	glfwSetInputMode(window, GLFW_STICKY_MOUSE_BUTTONS, 1);
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glfwWindowHint(GLFW_SAMPLES, 4);

	genTextListMap();

	printf("[INFO] gviewer initialized!!\n");
	printf("----------------------------------------------\n");
	printf(">>> Press 'T' to toggle tracking/world mode...\n");
	printf(">>> Press '1' to focus on the latest frame...\n");
	printf(">>> Hold 'left mouse' to rotate...\n");
	printf(">>> Hold 'right mouse' to translate...\n");
	printf("----------------------------------------------\n");
	const GLubyte *renderer = glGetString(GL_RENDERER);
	const GLubyte *version = glGetString(GL_VERSION);
	const GLubyte *vendor = glGetString(GL_VENDOR);
	const GLubyte *glslVersion = glGetString(GL_SHADING_LANGUAGE_VERSION);

	std::cout << "Renderer: " << renderer << std::endl;
	std::cout << "OpenGL version supported: " << version << std::endl;
	std::cout << "Vendor: " << vendor << std::endl;
	std::cout << "GLSL version: " << glslVersion << std::endl;

	/* Loop until the user closes the window */
	while (!glfwWindowShouldClose(window) && !mb_stop)
	{
#ifdef _WIN32
		Sleep(20);
#else
		usleep(20000);
#endif

		unique_lock<mutex> lock(m_mutex);
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Paint UI
		glColor3f(0.0, 0.0, 0.0);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		glColor3f(0.0, 0.0, 0.0);
		// if (mb_track)
		// 	showBottomText(" View mode: Track");
		// else
		// 	showBottomText(" View mode: World");
		// if (m_interval>=1)
		// 	showStatusText(to_string((int)m_interval)+" m  ");
		// else
		// 	showStatusText(to_string(m_interval) + " m  ");

		// Paint model
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		if (!mb_overlook)
			MyPerspective(45, (float)m_width / m_height, 0.01, 100.0);
		else
			glOrtho(-3.0 * m_width / m_height, 3.0 * m_width / m_height, -3, 3, -10, 1000);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		glPushMatrix();

		glTranslatef(m_trans_X, m_trans_Y, -m_dist * sqrt(3));

		if (!mb_overlook)
		{
			// gluLookAt(m_dist,m_dist,m_dist,0,0,0,0,0,1);
			glRotatef(-50 + m_rot_X, 1, 0, 0);
			glRotatef(-135, 0, 0, 1);
			glRotatef(0, 1, 0, 0);
			glRotatef(m_rot_Y, 0, 1, 0);
			glRotatef(m_rot_Z, 0, 0, 1);
		}
		else
		{
			// gluLookAt(0,0,m_dist,0,0,0,0,1,0);
		}

		m_interval = 0.1;

		glScalef(m_scale, m_scale, m_scale);

		if (!mb_track)
			glTranslatef(m_curr_X, m_curr_Y, m_curr_Z);

		double temp = 10 / m_scale;
		while (temp > 5)
		{
			temp /= 5;
			m_interval *= 5;
			if (temp > 2)
			{
				temp /= 2;
				m_interval *= 2;
			}
		}
		glPushMatrix();
		double xx = m_curr_X;
		double yy = m_curr_Y;
		double zz = m_curr_Z;
		if (mb_track && mv_frames.size() > 0 && mv_frames[0].size() > 0)
		{
			Eigen::Matrix4d mm;
			Eigen::Matrix4d mm_inv;
			mm.setIdentity();
			mm.block(0, 0, 3, 3) = mv_frames.front().front().first;
			mm.block(0, 3, 3, 1) = mv_frames.front().front().second;
			mm_inv = mm.inverse();
			glMultMatrixd(mm_inv.data());
			xx = -mm(0, 3);
			yy = -mm(1, 3);
			zz = -mm(2, 3);
		}

		if (mb_grid)
		{
			glPushMatrix();
			glTranslatef(-round(xx / m_interval) * m_interval,
						 -round(yy / m_interval) * m_interval,
						 -round((zz + 5) / m_interval) * m_interval);
			for (double i = 0; i < 100 * (1 / m_scale); i += m_interval)
			{
				double z = 1.5 * sqrt(sqrt(0.00001 * i / m_interval));
				if (i == 0)
					z = 1.5 * sqrt(sqrt(0.00001 * 1));
				glColor3f(0.70f + z, 0.70f + z, 0.70f + z);
				glLineWidth(1.0);
				glBegin(GL_LINES);
				glVertex3f(-100 * (1 / m_scale), i, 0);
				glVertex3f(100 * (1 / m_scale), i, 0);
				glEnd();
				glBegin(GL_LINES);
				glVertex3f(-100 * (1 / m_scale), -i, 0);
				glVertex3f(100 * (1 / m_scale), -i, 0);
				glEnd();
				glBegin(GL_LINES);
				glVertex3f(i, -100 * (1 / m_scale), 0);
				glVertex3f(i, 100 * (1 / m_scale), 0);
				glEnd();
				glBegin(GL_LINES);
				glVertex3f(-i, -100 * (1 / m_scale), 0);
				glVertex3f(-i, 100 * (1 / m_scale), 0);
				glEnd();
			}
			glPopMatrix();
		}
		if (mb_axis)
		{
			glPushMatrix();
			glTranslatef(-round(xx / m_curr_X) * m_interval,
						 -round(yy / m_curr_Y) * m_interval,
						 -round(zz / m_curr_Z) * m_interval);
			glPushMatrix();
			glColor3f(0.0f, 0.0f, 1.0f);
			MyCylinder(0.025f * 1 / m_scale, 1.5 * m_interval, 8);
			glPopMatrix();
			glPushMatrix();
			glRotatef(-90, 1, 0, 0);
			glColor3f(0.0f, 1.0f, 0.0f);
			MyCylinder(0.025f * 1 / m_scale, 1.5 * m_interval, 8);
			glPopMatrix();
			glPushMatrix();
			glRotatef(90, 0, 1, 0);
			glColor3f(1.0f, 0.0f, 0.0f);
			MyCylinder(0.025f * 1 / m_scale, 1.5 * m_interval, 8);
			glPopMatrix();
			glPopMatrix();
		}

		if (mb_trajetory)
		{
			float c_s[3] = {250 / 255.0, 167 / 255.0, 85 / 255.0};
			float c_e[3] = {80 / 255.0, 183 / 255.0, 193 / 255.0};
			glColor4f(c_s[0], c_s[1], c_s[2], 0.5);
			glLineWidth(6.0f);
			for (const auto &t : mv_trajectory)
			{
				glBegin(GL_LINE_STRIP);
				for (int i = 0; i < t.size(); i++)
				{
					auto p = t[i];
					int n = 20 - (t.size() - 1 - i);
					if (i > t.size() - 20 && i <= t.size() - 5)
					{
						glColor4f(
							c_s[0] + (c_e[0] - c_s[0]) / 15.0 * n,
							c_s[1] + (c_e[1] - c_s[1]) / 15.0 * n,
							c_s[2] + (c_e[2] - c_s[2]) / 15.0 * n, 0.5);
					}
					glVertex3d(p(0), p(1), p(2));
				}
				glEnd();
				if (mv_frames.back().size() > 0)
				{
					m_curr_X0 = -mv_frames.back().back().second[0];
					m_curr_Y0 = -mv_frames.back().back().second[1];
					m_curr_Z0 = -mv_frames.back().back().second[2];
				}
			}
		}
		if (mb_KFs)
		{
			for (const auto &fs : mv_frames)
			{
				for (const auto &f : fs)
				{
					Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity();
					Twb.block<3, 3>(0, 0) = f.first;
					Twb.block<3, 1>(0, 3) = f.second;
					glPushMatrix();

					glMultMatrixd(Twb.data());

					// MyAxis(1.0);
					MyAxisSimple(0.1);

					glPopMatrix();
				}
			}
		}
		if (mb_MPs)
		{
			double c_l[3] = {0.1, 0.1, 0};
			double c_h[3] = {0.1, 0.1, 1};
			double low = -5, high = 5;
			glPointSize(10.0f);
			for (const auto &pc : mv_pointCloud)
			{
				for (const auto &p : pc)
				{
					double h = (p(2) - (-5)) / 10;
					glColor3f(c_l[0] + (c_h[0] - c_l[0]) * h,
							  c_l[1] + (c_h[1] - c_l[1]) * h,
							  c_l[2] + (c_h[2] - c_l[2]) * h);
					glBegin(GL_POINTS);
					glVertex3f(p(0), p(1), p(2));
					glEnd();
				}
			}
		}

		if (mb_MPs)
		{
			glPointSize(0.5f);
			for (int i = 0; i < mv_pointCloud_semantic.size(); i++)
			{
				auto &pc = mv_pointCloud_semantic[i];
				if (i == 0)
					glColor3f(0 / 255.0, 128 / 255.0, 0 / 255.0);
				else if (i == 1)
					glColor3f(128 / 255.0, 128 / 255.0, 0 / 255.0);
				else if (i == 2)
					glColor3f(0 / 255.0, 0 / 255.0, 128 / 255.0);
				else if (i == 3)
					glColor3f(128 / 255.0, 0 / 255.0, 128 / 255.0);
				else if (i == 4)
					glColor3f(200 / 255.0, 0 / 255.0, 0 / 255.0);
				else if (i == 5)
					glColor3f(50 / 255.0, 50 / 255.0, 50 / 255.0);

				glBegin(GL_POINTS);
				for (const auto &p : pc)
				{
					glVertex3f(p(0), p(1), p(2));
				}
				glEnd();
			}
		}

		if (mb_instances)
		{
			for (int i = 0; i < mv_instances.size(); i++)
			{

				auto &instance = mv_instances[i];
				if (instance.type == VisualizedPatchType::BOX)
				{
#ifdef _WIN32
					Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity();
					Twb.block<3, 3>(0, 0) = instance.R;
					Twb.block<3, 1>(0, 3) = instance.t;
					glPushMatrix();

					glMultMatrixd(Twb.data());
					glColor3f(instance.color[0], instance.color[1], instance.color[2]);

					MyBox(instance.l, instance.w, instance.h);
					glPopMatrix();
#endif
				}
				else if (instance.type == VisualizedPatchType::LINE_SEGMENT)
				{
					// glLineWidth(3.0f);
#ifdef _WIN32
					glLineWidth(instance.linewidth);
					glBegin(GL_LINE_STRIP);
					for (int j = 0; j < instance.pts.size(); j++)
					{
						// glColor4f(instance.pts_color[j](0), instance.pts_color[j](1), instance.pts_color[j](2), 0.75);
						glColor4f(instance.pts_color[j](0), instance.pts_color[j](1), instance.pts_color[j](2), instance.alpha);
						glVertex3f(instance.pts[j](0), instance.pts[j](1), instance.pts[j](2));
					}
					glEnd();
#else
					if (instance.linewidth < 1.5)
					{
						glLineWidth(instance.linewidth);
						glBegin(GL_LINE_STRIP);
						for (int j = 0; j < instance.pts.size(); j++)
						{
							// glColor4f(instance.pts_color[j](0), instance.pts_color[j](1), instance.pts_color[j](2), 0.75);
							glColor4f(instance.pts_color[j](0), instance.pts_color[j](1), instance.pts_color[j](2), instance.alpha);
							glVertex3f(instance.pts[j](0), instance.pts[j](1), instance.pts[j](2));
						}
						glEnd();
					}
					else
					{
						glBegin(GL_TRIANGLE_STRIP);
						Eigen::Vector3d nn;
						for (int j = 0; j < instance.pts.size() - 1; j++)
						{
							// glColor4f(instance.pts_color[j](0), instance.pts_color[j](1), instance.pts_color[j](2), 0.75);
							glColor4f(instance.pts_color[j](0), instance.pts_color[j](1), instance.pts_color[j](2), instance.alpha);
							nn = (instance.pts[j + 1] - instance.pts[j]).normalized() * instance.linewidth / 200.0 / m_scale;

							glVertex3f(instance.pts[j](0) + nn(1), instance.pts[j](1) - nn(0), instance.pts[j](2));
							glVertex3f(instance.pts[j](0) - nn(1), instance.pts[j](1) + nn(0), instance.pts[j](2));
							glVertex3f(instance.pts[j + 1](0) + nn(1), instance.pts[j + 1](1) - nn(0), instance.pts[j](2));
							glVertex3f(instance.pts[j + 1](0) - nn(1), instance.pts[j + 1](1) + nn(0), instance.pts[j](2));
						}
						glEnd();
					}
				}
#endif

			}
		}

		glPopMatrix();
		glPopMatrix();

		/* Swap front and back buffers */
		glfwSwapBuffers(window);

		/* Poll for and process events */
		glfwPollEvents();

		if (need_screenshot)
		{
			need_screenshot = false;
			int mWidth, mHeight;
			glfwGetWindowSize(window, &mWidth, &mHeight);

			// glBufferLock.lock();
			std::cout << "\nSaving screenshot (" << mWidth << ", " << mHeight << ")\n";

			int n = 3 * mWidth * mHeight;
			GLubyte *pixels = new GLubyte[n];

			glPixelStorei(GL_PACK_ALIGNMENT, 1);

			glReadPixels(0, 0, mWidth, mHeight, GL_RGB, GL_UNSIGNED_BYTE, pixels);
			if (GL_NO_ERROR != glGetError())
				throw "Error: Unable to read pixels.";

			cv::Mat mm(mHeight, mWidth, CV_8UC3, pixels);
			cv::cvtColor(mm, mm, cv::COLOR_BGR2RGB);
			cv::imwrite(screenshot_path, mm);
			// Convert to FreeImage format & save to file
			// FIBITMAP* image = FreeImage_ConvertFromRawBits(pixels, mWidth, mHeight, 3 * mWidth, 24, 0xFF0000, 0x00FF00, 0x0000FF, false);
			// FreeImage_Save(FIF_BMP, image, "test.bmp", 0);

			//// Free resources
			// FreeImage_Unload(image);
			delete[] pixels;
			// glBufferLock.unlock();
		}
	}

	glfwTerminate();
	// cout<<window->
	// delete window;
}

void gviewer::ClearView()
{
	unique_lock<mutex> lock(m_mutex);
	mv_trajectory.clear();
	mv_pointCloud.clear();
}

void gviewer::SetFrames(const Frames &f)
{
	unique_lock<mutex> lock(m_mutex);
	mv_frames.clear();
	mv_frames.push_back(f);
}

void gviewer::SetPointCloud(const VPointCloud &pc)
{
	unique_lock<mutex> lock(m_mutex);
	mv_pointCloud.clear();
	mv_pointCloud.push_back(pc);
}

void gviewer::SetFrames(const vector<Frames> &vf)
{
	unique_lock<mutex> lock(m_mutex);
	mv_frames = vf;
}

void gviewer::AddNewFrame(const pair<Eigen::Matrix3d, Eigen::Vector3d> &f)
{
	unique_lock<mutex> lock(m_mutex);
	mv_frames.back().push_back(f);
}

void gviewer::SetPointCloud(const vector<VPointCloud> &vpc)
{
	unique_lock<mutex> lock(m_mutex);
	mv_pointCloud = vpc;
}

void gviewer::SetPointCloudSemantic(const vector<VPointCloud> &vpc)
{
	unique_lock<mutex> lock(m_mutex);
	mv_pointCloud_semantic = vpc;
}

void gviewer::SetInstances(const vector<VisualizedInstance> &vi)
{
	unique_lock<mutex> lock(m_mutex);
	mv_instances = vi;
}

void gviewer::AddNewPoint(const Eigen::Vector3d &p)
{
	unique_lock<mutex> lock(m_mutex);
	mv_pointCloud.back().push_back(p);
}

void gviewer::AddNewPoint(const vector<Eigen::Vector3d> &pts)
{
	unique_lock<mutex> lock(m_mutex);
	for (const auto &p : pts)
	{
		mv_pointCloud.back().push_back(p);
	}
}

void gviewer::SetTrajectory(const VPointCloud &t)
{
	unique_lock<mutex> lock(m_mutex);
	mv_trajectory.clear();
	mv_trajectory.push_back(t);
}

void gviewer::SetTrajectory(const vector<VPointCloud> &vt)
{
	unique_lock<mutex> lock(m_mutex);
	mv_trajectory = vt;
}

void gviewer::AddNewPos(const Eigen::Vector3d &p)
{
	unique_lock<mutex> lock(m_mutex);
	mv_trajectory.back().push_back(p);
}

void gviewer::ScreenShot(const string &s)
{
	unique_lock<mutex> lock(m_mutex);
	need_screenshot = true;
	screenshot_path = s;
}

void gviewer::SetCenter()
{
	unique_lock<mutex> lock(m_mutex);
}

void gviewer::Show()
{
	if (t)
		return;
	mb_stop = false;
	t = new thread(&gviewer::Run, this);
	// t->detach();
}

void gviewer::Hide()
{
	if (!t)
		return;

	mb_stop = true;
	// Sleep(2000);
	t->join();
	delete t;
	window = nullptr;
	t = nullptr;
}

void MyCylinder(GLdouble r, GLdouble l, int edgenum)
{
	for (int i = 0; i < edgenum; i++)
	{
		double a1 = i * 2 * 3.1416 / edgenum;
		double a2 = (i + 1) * 2 * 3.1416 / edgenum;

		double x1 = r * cos(a1);
		double y1 = r * sin(a1);
		double x2 = r * cos(a2);
		double y2 = r * sin(a2);
		glBegin(GL_TRIANGLES);
		glVertex3f(x1, y1, 0);
		glVertex3f(x2, y2, 0);
		glVertex3f(x1, y1, l);
		glEnd();

		glBegin(GL_TRIANGLES);
		glVertex3f(x1, y1, l);
		glVertex3f(x2, y2, l);
		glVertex3f(x2, y2, 0);
		glEnd();
	}
}

void MyPerspective(GLdouble fov, GLdouble aspectRatio, GLdouble zNear, GLdouble zFar)
{
	GLdouble rFov = fov * 3.14159265 / 180.0;
	glFrustum(-zNear * tan(rFov / 2.0) * aspectRatio,
			  zNear * tan(rFov / 2.0) * aspectRatio,
			  -zNear * tan(rFov / 2.0),
			  zNear * tan(rFov / 2.0),
			  zNear, zFar);
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods)
{

	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
	{
		m_mouseL_x = xpos;
		m_mouseL_y = ypos;
		m_rot_Z0 = m_rot_Z;
		m_rot_X0 = m_rot_X;
		is_pressL = true;
	}
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
	{
		m_mouseR_x = 0;
		m_mouseR_y = 0;
		is_pressL = false;
	}
	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS)
	{
		m_mouseR_x = xpos;
		m_mouseR_y = ypos;
		m_trans_X0 = m_trans_X;
		m_trans_Y0 = m_trans_Y;
		is_pressR = true;
	}
	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE)
	{
		m_mouseR_x = 0;
		m_mouseR_y = 0;
		is_pressR = false;
	}
}

static void cursor_position_callback(GLFWwindow *window, double xpos, double ypos)
{
	if (is_pressL)
	{
		m_rot_Z = m_rot_Z0 + (xpos - m_mouseL_x) * 0.2;
		m_rot_X = m_rot_X0 + (ypos - m_mouseL_y) * 0.2;
	}
	if (is_pressR)
	{
		m_trans_Y = m_trans_Y0 - (ypos - m_mouseR_y) * 0.015;
		m_trans_X = m_trans_X0 + (xpos - m_mouseR_x) * 0.015;
	}
}

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
	if (yoffset > 0)
		m_scale *= 1.1;
	if (yoffset < 0)
		m_scale /= 1.1;
}
void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_O && action == GLFW_PRESS)
		mb_overlook = (bool)(1 - (int)mb_overlook);
	if (key == GLFW_KEY_T && action == GLFW_PRESS)
	{
		mb_track = (bool)(1 - (int)mb_track);
	}
	if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
	{
		m_trans_X = 0;
		m_trans_Y = 0;
		m_rot_X = 0;
		m_rot_Y = 0;
		m_scale = 1;
	}
	if (key == GLFW_KEY_1 && action == GLFW_PRESS)
	{
		m_trans_X = 0;
		m_trans_Y = 0;
		if (!mb_track)
		{
			m_curr_X = m_curr_X0;
			m_curr_Y = m_curr_Y0;
			m_curr_Z = m_curr_Z0;
		}
	}
	if (key == GLFW_KEY_W && action == GLFW_PRESS)
	{
		m_curr_Y -= m_interval;
	}
	if (key == GLFW_KEY_S && action == GLFW_PRESS)
	{
		m_curr_Y += m_interval;
	}
	if (key == GLFW_KEY_A && action == GLFW_PRESS)
	{
		m_curr_X += m_interval;
	}
	if (key == GLFW_KEY_D && action == GLFW_PRESS)
	{
		m_curr_X -= m_interval;
	}
}

void MyAerial(GLfloat size)
{
	Eigen::Matrix4d T;
	/*T << 0, 0, 1, 0,
		0, 1, 0, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;*/
	T = Eigen::Matrix4d::Identity();
	glPushMatrix();

	glMultMatrixd(T.data());

	float h = size / 2;
	float l = sqrt(2) * size;
	float r = size * 0.7;
	glColor3f(0.2, 0.2, 0.8);
	glLineWidth(4.0f);
	glBegin(GL_LINES);
	glVertex3d(0, 0, 0);
	glVertex3d(size, size, h);
	glVertex3d(0, 0, 0);
	glVertex3d(-size, -size, h);
	glVertex3d(0, 0, 0);
	glVertex3d(-size, size, h);
	glVertex3d(0, 0, 0);
	glVertex3d(size, -size, h);
	glEnd();
	for (int i = 0; i < 4; i++)
	{
		double dd1 = 90.0 / 180 * 3.1415926;
		int n = 20;
		double dd2 = 360.0 / n / 180 * 3.1415926;
		glBegin(GL_LINE_STRIP);
		for (int j = 0; j <= n; j++)
		{
			glVertex3d(cos(dd1 * (i + 0.5)) * l + cos(dd2 * j) * r, sin(dd1 * (i + 0.5)) * l + sin(dd2 * j) * r, h);
		}
		glEnd();
	}
	glPopMatrix();
}

void MyFrame()
{
	float w = 0.1;
	float h = 0.1;
	float z = w * 0.6;
	glLineWidth(2);
	glColor3f(0.3f, 0.3f, 1.0f);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(w, h, z);
	glVertex3f(0, 0, 0);
	glVertex3f(w, -h, z);
	glVertex3f(0, 0, 0);
	glVertex3f(-w, -h, z);
	glVertex3f(0, 0, 0);
	glVertex3f(-w, h, z);

	glVertex3f(w, h, z);
	glVertex3f(w, -h, z);

	glVertex3f(-w, h, z);
	glVertex3f(-w, -h, z);

	glVertex3f(-w, h, z);
	glVertex3f(w, h, z);

	glVertex3f(-w, -h, z);
	glVertex3f(w, -h, z);
	glEnd();
}

void MyAxis(GLfloat scale)
{
	glPushMatrix();
	glColor3f(0.5f, 0.5f, 1.0f);
	MyCylinder(0.025f * 1 / m_scale, 0.5 * m_interval * scale, 8);
	glPopMatrix();
	glPushMatrix();
	glRotatef(-90, 1, 0, 0);
	glColor3f(0.5f, 1.0f, 0.5f);
	MyCylinder(0.025f * 1 / m_scale, 0.5 * m_interval * scale, 8);
	glPopMatrix();
	glPushMatrix();
	glRotatef(90, 0, 1, 0);
	glColor3f(1.0f, 0.5f, 0.5f);
	MyCylinder(0.025f * 1 / m_scale, 0.5 * m_interval * scale, 8);
	glPopMatrix();
}

void MyAxisSimple(GLfloat scale)
{
	glColor3f(0.5f, 0.5f, 1.0f);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 1);
	glEnd();
	glColor3f(0.5f, 1.0f, 0.5f);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 1, 0);
	glEnd();
	glColor3f(1.0f, 0.5f, 0.5f);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(1, 0, 0);
	glEnd();
}

void genTextListMap()
{
	map<char, uint64_t> letter_map = {
		{'1', 0x7e1818181c181800}, {'2', 0x7e060c3060663c00}, {'3', 0x3c66603860663c00}, {'4', 0x30307e3234383000}, {'5', 0x3c6660603e067e00}, {'6', 0x3c66663e06663c00}, {'7', 0x1818183030667e00}, {'8', 0x3c66663c66663c00}, {'9', 0x3c66607c66663c00}, {'0', 0x3c66666e76663c00}, {'A', 0x6666667e66663c00}, {'B', 0x3e66663e66663e00}, {'C', 0x3c66060606663c00}, {'D', 0x3e66666666663e00}, {'E', 0x7e06063e06067e00}, {'F', 0x0606063e06067e00}, {'G', 0x3c66760606663c00}, {'H', 0x6666667e66666600}, {'I', 0x3c18181818183c00}, {'J', 0x1c36363030307800}, {'K', 0x66361e0e1e366600}, {'L', 0x7e06060606060600}, {'M', 0xc6c6c6d6feeec600}, {'N', 0xc6c6e6f6decec600}, {'O', 0x3c66666666663c00}, {'P', 0x06063e6666663e00}, {'Q', 0x603c766666663c00}, {'R', 0x66361e3e66663e00}, {'S', 0x3c66603c06663c00}, {'T', 0x18181818185a7e00}, {'U', 0x7c66666666666600}, {'V', 0x183c666666666600}, {'W', 0xc6eefed6c6c6c600}, {'X', 0xc6c66c386cc6c600}, {'Y', 0x1818183c66666600}, {'Z', 0x7e060c1830607e00}, {'a', 0x7c667c603c000000}, {'b', 0x3e66663e06060600}, {'c', 0x3c6606663c000000}, {'d', 0x7c66667c60606000}, {'e', 0x3c067e663c000000}, {'f', 0x0c0c3e0c0c6c3800}, {'g', 0x3c607c66667c0000}, {'h', 0x6666663e06060600}, {'i', 0x3c18181800180000}, {'j', 0x1c36363030003000}, {'k', 0x66361e3666060600}, {'l', 0x1818181818181800}, {'m', 0xd6d6feeec6000000}, {'n', 0x6666667e3e000000}, {'o', 0x3c6666663c000000}, {'p', 0x06063e66663e0000}, {'q', 0xf0b03c36363c0000}, {'r', 0x060666663e000000}, {'s', 0x3e403c027c000000}, {'t', 0x1818187e18180000}, {'u', 0x7c66666666000000}, {'v', 0x183c666600000000}, {'w', 0x7cd6d6d6c6000000}, {'x', 0x663c183c66000000}, {'y', 0x3c607c6666000000}, {'z', 0x3c0c18303c000000}, {' ', 0x0000000000000000}, {':', 0x0018180018180000}, {'(', 0x6030181818306000}, {')', 0x060c1818180c0600}, {'[', 0x7818181818187800}, {']', 0x1e18181818181e00}, {'.', 0x0606000000000000}};

	for (auto iter = letter_map.begin(); iter != letter_map.end(); iter++)
	{
		GLuint index = glGenLists(iter->first);
		glNewList(index, GL_COMPILE);
		for (int i = 0; i < 64; i++)
		{
			int xxx = i % 8;
			int yyy = 8 - i / 8;
			if (iter->second >> i & 1)
			{
				glBegin(GL_POLYGON);
				glVertex2f(xxx, yyy);
				glVertex2f((xxx + 1), yyy);
				glVertex2f((xxx + 1), (yyy + 1));
				glVertex2f(xxx, (yyy + 1));
				glEnd();
			}
		}
		glEndList();
		m_text_call_map[iter->first] = index;
	}
}

void showBottomText(string s)
{
	glPushMatrix();
	glTranslatef(-1, -1, 0.0);
	glScalef((float)300.0 / m_width, 300.0 / m_height, 1);
	for (int i = 0; i < s.size(); i++)
	{
		glPushMatrix();
		glTranslatef(0.07 * i, 0.0, 0.0);
		glScalef(0.01, 0.01, 1);
		glCallList(m_text_call_map[s[i]]);
		glPopMatrix();
	}
	glPopMatrix();
}

void showStatusText(string s)
{
	glPushMatrix();
	glTranslatef(1, -1, 0.0);
	glScalef((float)300.0 / m_width, 300.0 / m_height, 1);
	glTranslatef(-0.07 * s.size(), 0, 0.0);
	for (int i = 0; i < s.size(); i++)
	{
		glPushMatrix();
		glTranslatef(0.07 * i, 0.0, 0.0);
		glScalef(0.01, 0.01, 1);
		glCallList(m_text_call_map[s[i]]);
		glPopMatrix();
	}
	glPopMatrix();
}

void MyBox(GLfloat l, GLfloat w, GLfloat h)
{
	// glLineWidth(3);
	glLineWidth(0.5);
	glBegin(GL_LINE_LOOP);
	glVertex3f(l / 2, w / 2, h / 2);
	glVertex3f(-l / 2, w / 2, h / 2);
	glVertex3f(-l / 2, -w / 2, h / 2);
	glVertex3f(l / 2, -w / 2, h / 2);
	glEnd();
	glBegin(GL_LINE_LOOP);
	glVertex3f(l / 2, w / 2, -h / 2);
	glVertex3f(-l / 2, w / 2, -h / 2);
	glVertex3f(-l / 2, -w / 2, -h / 2);
	glVertex3f(l / 2, -w / 2, -h / 2);
	glEnd();

	glBegin(GL_LINES);
	glVertex3f(l / 2, w / 2, h / 2);
	glVertex3f(l / 2, w / 2, -h / 2);

	glVertex3f(-l / 2, w / 2, h / 2);
	glVertex3f(-l / 2, w / 2, -h / 2);

	glVertex3f(-l / 2, -w / 2, h / 2);
	glVertex3f(-l / 2, -w / 2, -h / 2);

	glVertex3f(l / 2, -w / 2, h / 2);
	glVertex3f(l / 2, -w / 2, -h / 2);
	glEnd();
}