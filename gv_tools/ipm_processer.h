#ifndef IPM_PROCESSER_H
#define IPM_PROCESSER_H

#include "gv_utils.h"
#include <exception>
#include <fstream>

namespace gv
{
	enum IPMType
	{
		NORMAL = 0
	};

	class IPMProcesser
	{
	public:
		IPMProcesser(CameraConfig conf, IPMType ipm_type);
		~IPMProcesser();

		/**
		 * Update the camera-ground geometric parameters and generate the IPM mapping matrix.
		 * 
		 * @param cg Camera-ground geometric parameters.
		 * @return 0 if no exception.
		 */
		int updateCameraGroundGeometry(CameraGroundGeometry cg);

		/**
		 * Generate the IPM image.
		 * 
		 * @param mm Input perspective image.
		 * @param need_undistort If the input image needs undistortion or has been undistorted beforehand.
		 * @param mm_undist Output the undistorted perspective image.
		 * @return IPM image.
		 */
		cv::Mat genIPM(cv::Mat mm, bool need_undistort = true, cv::Mat mm_undist = cv::Mat()) const;

		/**
		 * Auxiliary function for image undistortion, since the IPMprecessor handles all camera parameters.
		 * 
		 * @param mm Input perspective image.
		 * @return Undistorted perspective image.
		 */
		cv::Mat genUndistortedImage(cv::Mat mm) const;
		cv::Vec4d getUndistortedIntrinsics();

		/**
		 * Point-to-point transformation.
		 * 
		 * @param p 2-D point or 3-D point.
		 * @param cg_temp Optional camera-ground geometry for temporary use.
		 * @return Transformed point.
		 */
		cv::Point2f IPM2Perspective(cv::Point2f p, CameraGroundGeometry *cg_temp = nullptr) const;
		cv::Point2f Perspective2IPM(cv::Point2f p, CameraGroundGeometry *cg_temp = nullptr) const;
		Eigen::Vector3d Perspective2Metric(cv::Point2f p, CameraGroundGeometry *cg_temp = nullptr) const;
		Eigen::Vector3d NormalizedPerspective2Metric(Eigen::Vector3d p, CameraGroundGeometry *cg_temp = nullptr) const;
		Eigen::Vector3d IPM2Metric(cv::Point2f p, CameraGroundGeometry *cg_temp = nullptr) const;
		cv::Point2f Metric2IPM(Eigen::Vector3d p, CameraGroundGeometry *cg_temp = nullptr) const;

		/**
		 * Generate a coarse mask for ground feature extraction, according to CameraConfig.
		 * 
		 * @return 0-1 Mask;
		 */
		cv::Mat guessGroundROI();

		int updateIPMMap();

		CameraConfig _config;

		IPMType _ipm_type;
		double _fx, _fy, _cx, _cy, _h, _w; // for undistorted images
		cv::Mat _undist_M0, _undist_M1;

		// for pointwise IPM mapping (only for debug use)
		float *_IPM_mapx_array = nullptr;
		float *_IPM_mapy_array = nullptr;
		cv::Mat _IPM_mapx;
		cv::Mat _IPM_mapy;

		// for perspective-matrix-based IPM mapping
		cv::Mat _IPM_transform;

		// online camera-ground parameters, updated by updateCameraGroundGeometry()
		Eigen::Matrix3d _Rc0c1; // R^{camera}_{virtual camera}
		double _d;
	};

}
#endif