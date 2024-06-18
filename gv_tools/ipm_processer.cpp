#include "ipm_processer.h"
#include <camodocal/camera_models/CataCamera.h>
#include <camodocal/camera_models/EquidistantCamera.h>
#include <camodocal/camera_models/PinholeCamera.h>
#include <camodocal/camera_models/PinholeFullCamera.h>
#include <camodocal/camera_models/ScaramuzzaCamera.h>

//#define POINTWISE_MAPPING
namespace gv
{
	IPMProcesser::IPMProcesser(CameraConfig conf, IPMType ipm_type) : _config(conf), _ipm_type(ipm_type) {
#ifdef POINTWISE_MAPPING
		_IPM_mapx_array = new float[_config.IPM_HEIGHT*_config.IPM_WIDTH];
		_IPM_mapy_array = new float[_config.IPM_HEIGHT*_config.IPM_WIDTH];

		memset(_IPM_mapx_array, 0.0, _config.IPM_HEIGHT*_config.IPM_WIDTH * sizeof(float));
		memset(_IPM_mapy_array, 0.0, _config.IPM_HEIGHT*_config.IPM_WIDTH * sizeof(float));

		_IPM_mapx = cv::Mat(_config.IPM_HEIGHT, _config.IPM_WIDTH, CV_32FC1);
		_IPM_mapy = cv::Mat(_config.IPM_HEIGHT, _config.IPM_WIDTH, CV_32FC1);
#endif

		// config.camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile("H:/2022_10_12/data_20221012133020/config/cam0_pinhole.yaml");
		if (_config.camera->modelType() == camodocal::Camera::PINHOLE)
		{
			_h = _config.camera->imageHeight();
			_w = _config.camera->imageWidth();
			_fx = std::dynamic_pointer_cast<camodocal::PinholeCamera>(_config.camera)->getParameters().fx();
			_fy = std::dynamic_pointer_cast<camodocal::PinholeCamera>(_config.camera)->getParameters().fy();
			_cx = std::dynamic_pointer_cast<camodocal::PinholeCamera>(_config.camera)->getParameters().cx();
			_cy = std::dynamic_pointer_cast<camodocal::PinholeCamera>(_config.camera)->getParameters().cy();
		}
		else if (_config.camera->modelType() == camodocal::Camera::PINHOLE_FULL
			|| _config.camera->modelType() == camodocal::Camera::KANNALA_BRANDT
			|| _config.camera->modelType() == camodocal::Camera::SCARAMUZZA
			|| _config.camera->modelType() == camodocal::Camera::MEI)
		{
			_h = _config.camera->imageHeight();
			_w = _config.camera->imageWidth();
			Eigen::Vector2d p0(0, 0);
			Eigen::Vector2d p1(_w - 1, _h - 1);
			Eigen::Vector3d p0_, p1_;
			_config.camera->liftProjective(p0, p0_);
			_config.camera->liftProjective(p1, p1_);
			p0_ /= p0_(2);
			p1_ /= p1_(2);
			_fx = (_w - 1) / (p1_(0) - p0_(0));
			_fy = (_h - 1) / (p1_(1) - p0_(1));
			_cx = -_fx * p0_(0);
			_cy = -_fy * p0_(1);
		}
		else
		{
			throw std::exception();
		}
		_config.camera->initUndistortRectifyMap(_undist_M0, _undist_M1, _fx, _fy, cv::Size(_w, _h), _cx, _cy);
		this->updateCameraGroundGeometry(_config.cg);
	}
	IPMProcesser::~IPMProcesser()
	{
		if (_IPM_mapx_array) delete[] _IPM_mapx_array;
		if (_IPM_mapy_array) delete[] _IPM_mapy_array;
	}

    int IPMProcesser::updateCameraGroundGeometry(CameraGroundGeometry cg)
    {
		_Rc0c1 = cg.getR();
		_d = cg.getH();
		return updateIPMMap();
    }

    cv::Mat IPMProcesser::guessGroundROI()
    {
		cv::Point p0 = IPM2Perspective(cv::Point2f(0,0));
		cv::Point p1 = IPM2Perspective(cv::Point2f(_config.IPM_WIDTH-1,0));
		cv::Point p2 = IPM2Perspective(cv::Point2f(_config.IPM_WIDTH-1,_config.IPM_HEIGHT/2));
		cv::Point p3 = IPM2Perspective(cv::Point2f(0,_config.IPM_HEIGHT/2));
		std::vector<cv::Point> pts({p0,p1,p2,p3});
		cv::Mat mask = cv::Mat(_config.camera->imageHeight(), _config.camera->imageWidth(), CV_8UC1);
		mask.setTo(0);
		cv::fillConvexPoly(mask,pts,255);
        return mask;
    }

    int IPMProcesser::updateIPMMap()
    {
#ifdef POINTWISE_MAPPING
		double c1_p_c1f_array[3];
		double R_c0c1_array[3][3] = { {_Rc0c1(0, 0),_Rc0c1(0, 1),_Rc0c1(0, 2)},
									  {_Rc0c1(1, 0),_Rc0c1(1, 1),_Rc0c1(1, 2)},
									  {_Rc0c1(2, 0),_Rc0c1(2, 1),_Rc0c1(2, 2)} };
		double xy1_y_array[3];
		if (_ipm_type == IPMType::NORMAL)
		{
			for (int i = 0; i < _config.IPM_HEIGHT; i++)
			{
				for (int j = 0; j < _config.IPM_WIDTH; j++)
				{
					c1_p_c1f_array[0] = (j - _config.IPM_WIDTH / 2) * _config.IPM_RESO;
					c1_p_c1f_array[1] = _d;
					c1_p_c1f_array[2] =(_config.IPM_HEIGHT - i - 1) * _config.IPM_RESO;
					xy1_y_array[0] = (R_c0c1_array[0][0] * c1_p_c1f_array[0] + R_c0c1_array[0][1] * c1_p_c1f_array[1] + R_c0c1_array[0][2] * c1_p_c1f_array[2]);
					xy1_y_array[1] = (R_c0c1_array[1][0] * c1_p_c1f_array[0] + R_c0c1_array[1][1] * c1_p_c1f_array[1] + R_c0c1_array[1][2] * c1_p_c1f_array[2]);
					xy1_y_array[2] = (R_c0c1_array[2][0] * c1_p_c1f_array[0] + R_c0c1_array[2][1] * c1_p_c1f_array[1] + R_c0c1_array[2][2] * c1_p_c1f_array[2]);
					_IPM_mapx_array[i*_config.IPM_WIDTH + j] = _fx * xy1_y_array[0] / xy1_y_array[2] + _cx;
					_IPM_mapy_array[i*_config.IPM_WIDTH + j] = _fy * xy1_y_array[1] / xy1_y_array[2] + _cy;
				}
			}
			_IPM_mapx = cv::Mat(_config.IPM_HEIGHT, _config.IPM_WIDTH, CV_32FC1, _IPM_mapx_array);
			_IPM_mapy = cv::Mat(_config.IPM_HEIGHT, _config.IPM_WIDTH, CV_32FC1, _IPM_mapy_array);
			return 0;
		}
		else
		{
			return -1;
		}

#else
		Eigen::Matrix3d K, K_IPM_inv;
		K << _fx, 0, _cx,
			0, _fy, _cy,
			0, 0, 1;
		if (_ipm_type == IPMType::NORMAL)
		{
			K_IPM_inv << _config.IPM_RESO / _d, 0, (-_config.IPM_WIDTH / 2)*_config.IPM_RESO / _d,
				            0, 0, 1,
				         0, -_config.IPM_RESO / _d, (_config.IPM_HEIGHT - 1)*_config.IPM_RESO / _d;
			Eigen::Matrix3d T_IPM = (K * _Rc0c1 * _d * K_IPM_inv).inverse();
			cv::eigen2cv(T_IPM, _IPM_transform);
			_IPM_transform.convertTo(_IPM_transform, CV_32F);
			return 0;
		}
		else
		{
			return -1;
		}
#endif
	}

	cv::Mat IPMProcesser::genUndistortedImage(cv::Mat mm) const
	{
		cv::Mat mm_undist;
		cv::remap(mm, mm_undist, _undist_M0, _undist_M1, cv::INTER_NEAREST);
		return mm_undist;
	}

	cv::Mat IPMProcesser::genIPM(cv::Mat mm, bool need_undistort, cv::Mat mm_undist)  const
	{
		if (need_undistort)
		{
			cv::remap(mm, mm_undist, _undist_M0, _undist_M1, cv::INTER_NEAREST);
		}
		else
		{
			mm_undist = mm;
		}
		cv::Mat m_ipm(_config.IPM_HEIGHT, _config.IPM_WIDTH, CV_8U);
#ifdef POINTWISE_MAPPING
		cv::remap(mm_undist, m_ipm, _IPM_mapx, _IPM_mapy, cv::INTER_NEAREST);
		cv::remap(mm_undist, m_ipm, _IPM_mapx, _IPM_mapy, cv::INTER_LINEAR);
#else
		cv::warpPerspective(mm_undist, m_ipm, _IPM_transform, m_ipm.size(), cv::INTER_NEAREST);
#endif
		return m_ipm;
	}
	
	cv::Vec4d IPMProcesser::getUndistortedIntrinsics()
	{
		return cv::Vec4d(_fx, _fy, _cx, _cy);
	}

	cv::Point2f IPMProcesser::IPM2Perspective(cv::Point2f p, CameraGroundGeometry* cg_temp)  const
	{
		Eigen::Matrix3d Rc0c1 = _Rc0c1;
		double d = _d;
		if(cg_temp!=nullptr)
		{
			Rc0c1 = cg_temp->getR();
			d = cg_temp->getH();
		}

		if (_ipm_type == IPMType::NORMAL)
		{
			Eigen::Vector3d pc1f = Eigen::Vector3d((p.x - _config.IPM_WIDTH / 2) * _config.IPM_RESO,
				d,
				(_config.IPM_HEIGHT - p.y - 1) * _config.IPM_RESO);
			Eigen::Vector3d pc0f = Rc0c1 * pc1f;
			Eigen::Vector3d uv_c0 = Eigen::Vector3d(pc0f(0) / pc0f(2) *_fx + _cx, pc0f(1) / pc0f(2)*_fy + _cy, 1);
			return cv::Point2f(uv_c0(0), uv_c0(1));
		}
		else
		{
			throw std::exception();
		}
	}

	cv::Point2f IPMProcesser::Perspective2IPM(cv::Point2f p, CameraGroundGeometry* cg_temp)  const
	{
		Eigen::Matrix3d Rc0c1 = _Rc0c1;
		double d = _d;
		if(cg_temp!=nullptr)
		{
			Rc0c1 = cg_temp->getR();
			d = cg_temp->getH();
		}

		double x = (p.x - _cx) / _fx;
		double y = (p.y - _cy) / _fy;

		cv::Point2f pipm;
		if (_ipm_type == IPMType::NORMAL)
		{
			Eigen::Vector3d xyzc1f = (Rc0c1.transpose()*Eigen::Vector3d(x, y, 1));
			double depth = d / xyzc1f(1);
			//std::cerr << depth << std::endl;
			Eigen::Vector3d pc1f = xyzc1f * depth;
			//std::cerr << pc1f.transpose() << std::endl;
			pipm = cv::Point2f(pc1f.x() / _config.IPM_RESO + _config.IPM_WIDTH / 2,
				              -pc1f.z() / _config.IPM_RESO + _config.IPM_HEIGHT - 1);
			return  pipm;
		}
		else
		{
			throw std::exception();
		}
	}

	Eigen::Vector3d IPMProcesser::Perspective2Metric(cv::Point2f p, CameraGroundGeometry* cg_temp) const
	{
		Eigen::Matrix3d Rc0c1 = _Rc0c1;
		double d = _d;
		if(cg_temp!=nullptr)
		{
			Rc0c1 = cg_temp->getR();
			d = cg_temp->getH();
		}

		double x = (p.x - _cx) / _fx;
		double y = (p.y - _cy) / _fy;

		Eigen::Vector3d pc0f;
		if (_ipm_type == IPMType::NORMAL)
		{
			Eigen::Vector3d xyzc1f = (Rc0c1.transpose()*Eigen::Vector3d(x, y, 1));
			double depth = d / xyzc1f(1);
			//std::cerr << depth << std::endl;
			Eigen::Vector3d pc1f = xyzc1f * depth;
			pc0f = Rc0c1 * pc1f;
			return  pc0f;
		}
		else
		{
			throw std::exception();
		}
	}

	Eigen::Vector3d IPMProcesser::NormalizedPerspective2Metric(Eigen::Vector3d p, CameraGroundGeometry* cg_temp) const
	{
		return Perspective2Metric(cv::Point2f(p.x()*_fx + _cx, p.y()*_fy + _cy), cg_temp);
	}

	Eigen::Vector3d IPMProcesser::IPM2Metric(cv::Point2f p, CameraGroundGeometry* cg_temp) const
	{
		Eigen::Matrix3d Rc0c1 = _Rc0c1;
		double d = _d;
		if(cg_temp!=nullptr)
		{
			Rc0c1 = cg_temp->getR();
			d = cg_temp->getH();
		}

		if (_ipm_type == IPMType::NORMAL)
		{
			Eigen::Vector3d pc1f = Eigen::Vector3d((p.x - _config.IPM_WIDTH / 2) * _config.IPM_RESO,
				                                   d,
				                                   (_config.IPM_HEIGHT - p.y - 1) * _config.IPM_RESO);
			Eigen::Vector3d pc0f = Rc0c1 * pc1f;
			return pc0f;
		}
		else
		{
			throw std::exception();
		}
	}

	cv::Point2f IPMProcesser::Metric2IPM(Eigen::Vector3d p, CameraGroundGeometry* cg_temp) const
	{
		Eigen::Matrix3d Rc0c1 = _Rc0c1;
		double d = _d;
		if(cg_temp!=nullptr)
		{
			Rc0c1 = cg_temp->getR();
			d = cg_temp->getH();
		}

		cv::Point2f uv;
		if (_ipm_type == IPMType::NORMAL)
		{
			Eigen::Vector3d pc1f = Rc0c1.transpose() * p;
			uv.x = pc1f(0) / _config.IPM_RESO + _config.IPM_WIDTH / 2;
			uv.y = _config.IPM_HEIGHT - 1 - pc1f(2) / _config.IPM_RESO;
			return uv;
		}
		else
		{
			throw std::exception();
		}
	}
}

	