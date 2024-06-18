#include "gv_utils.h"

namespace gv
{

    CameraGroundGeometry::CameraGroundGeometry(const double &a, const double &t, const double &h)
    {
        this->update(a, t, h);
    }

    CameraGroundGeometry::CameraGroundGeometry(const Eigen::Vector3d &n, const double &h)
    {
        this->update(n, h);
    }

    CameraGroundGeometry::CameraGroundGeometry(const Eigen::Matrix3d &R_c_cg, const double &h)
    {
        this->update(R_c_cg, h);
    }

    double CameraGroundGeometry::getAlpha() const
    {
        return _alpha * 180 / M_PI;
    }

    double CameraGroundGeometry::getTheta() const
    {
        return _theta * 180 / M_PI;
    }

    double CameraGroundGeometry::getH() const
    {
        return _h;
    }

    Eigen::Matrix3d CameraGroundGeometry::getR() const
    {
        return _R_c_cg;
    }

    Eigen::Vector3d CameraGroundGeometry::getN() const
    {
        return _n;
    }

    void CameraGroundGeometry::update(const double &a, const double &t, const double &h)
    {
        if (h > 0)
            _h = h;

        _alpha = a / 180 * M_PI;
        _theta = t / 180 * M_PI;

        Eigen::Matrix3d Ra;
        Ra << cos(_alpha), -sin(_alpha), 0,
            sin(_alpha), cos(_alpha), 0,
            0, 0, 1;
        Eigen::Matrix3d Rt;
        Rt << 1, 0, 0,
            0, cos(_theta), -sin(_theta),
            0, sin(_theta), cos(_theta);
        _R_c_cg = Ra * Rt;
        _n = _R_c_cg * Eigen::Vector3d(0, -1, 0);
    }

    void CameraGroundGeometry::update(const Eigen::Vector3d &n, const double &h)
    {
        Eigen::Matrix3d R_c_cg = Eigen::Quaterniond::FromTwoVectors(n.normalized(), Eigen::Vector3d(0, -1, 0)).toRotationMatrix();
        this->update(R_c_cg, h);
    }

    void CameraGroundGeometry::update(const Eigen::Matrix3d &R_c_cg, const double &h)
    {
        double a = atan2(R_c_cg(1, 0),R_c_cg(0, 0)) * 180 / M_PI;
        double b = atan2(R_c_cg(2, 1), R_c_cg(2, 2)) * 180 / M_PI;
        this->update(a, b, h);
    }

    bool CameraGroundGeometry::operator==(const CameraGroundGeometry c1) const
    {
        if (this->getAlpha() == c1.getAlpha() && this->getTheta() == c1.getTheta() && this->getH() == c1.getH())
            return true;
        else
            return false;
    }

    CameraConfig::CameraConfig(const std::string &config_file)
    {
        cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
        IPM_HEIGHT = fsSettings["IPM_HEIGHT"];
        IPM_WIDTH = fsSettings["IPM_WIDTH"];
        IPM_RESO = fsSettings["IPM_RESO"];
        
        cg = CameraGroundGeometry(fsSettings["priori_alpha"], fsSettings["priori_theta"], fsSettings["priori_H"]);


        cv::Mat Tic_mat;
        fsSettings["body_T_cam0"] >> Tic_mat;
        cv::cv2eigen(Tic_mat, Tic);
        Ric = Tic.block(0, 0, 3, 3);
        tic = Tic.block(0, 3, 3, 1);

        int pn = config_file.find_last_of('/');
        std::string configPath = config_file.substr(0, pn);

        std::string cam0Calib;
        fsSettings["cam0_calib"] >> cam0Calib;
        std::string cam0Path = configPath + "/" + cam0Calib;

        camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cam0Path);

        fsSettings.release();

    }
}
