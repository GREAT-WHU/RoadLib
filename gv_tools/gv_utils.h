#ifndef GV_UTILS_H
#define GV_UTILS_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <camodocal/camera_models/CameraFactory.h>
#include <camodocal/gpl/gpl.h>

namespace gv
{
	/**
	 * @brief Camera-ground geometry which parameterizes the ground plane in the camera frame.
	 *
	 * Note that there are different representations of the camera-ground geometry.
	 * 1) A height with a two-step rotation (alpha and theta) to make the camera frame parallel with the ground.
	 * 2) A height with a rotation matrix (R^c_cg, where cg is the virtual camera frame). 
	 * 3) A height with a normal vector of the ground plane.
	 * 
	 */
	class CameraGroundGeometry
	{
	public:
		CameraGroundGeometry(){};
		CameraGroundGeometry(const double &a, const double &t, const double &h);
		CameraGroundGeometry(const Eigen::Vector3d &n, const double &h);
		CameraGroundGeometry(const Eigen::Matrix3d &R_c_cg, const double &h);

		double getAlpha() const;
		double getTheta() const;
		double getH() const;
		Eigen::Matrix3d getR() const; // R^{camera}_{virtual camera}
		Eigen::Vector3d getN() const; // n^{camera}_{ground norm}

		void update(const double &a, const double &t, const double &h = -1);
		void update(const Eigen::Vector3d &n, const double &h = -1);
		void update(const Eigen::Matrix3d &R_c_cg, const double &h = -1);
		bool operator==(const CameraGroundGeometry c1) const;

	private:
		double _alpha;
		double _theta;
		double _h;

		Eigen::Matrix3d _R_c_cg; // R^{camera}_{virtual camera}

		Eigen::Vector3d _n; // n^{camera}_{ground norm}
	};


	/**
	 * @brief Configuration struct which contains camera parameters and IPM settings.
	 *
	 * The camera models from camodocal are used.
	 * 
	 */
	struct CameraConfig
	{
	public:
		CameraConfig(){};
		CameraConfig(const std::string &config_file);

	public:
		// IPM settings.
		int IPM_HEIGHT;
		int IPM_WIDTH;
		float IPM_RESO;
		int RAW_RESIZE;

		// Extrinsic parameters.
		Eigen::Matrix3d Ric;
		Eigen::Vector3d tic;
		Eigen::Matrix4d Tic;

		camodocal::CameraPtr camera; // Camera model.
		CameraGroundGeometry cg; // Prior camera-ground geometry.
	};
}
#endif