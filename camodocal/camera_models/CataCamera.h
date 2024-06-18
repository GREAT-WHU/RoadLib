#ifndef CATACAMERA_H
#define CATACAMERA_H

#include <opencv2/core/core.hpp>
#include <string>

#include "Camera.h"

namespace camodocal
{

/**
 * C. Mei, and P. Rives, Single View Point Omnidirectional Camera Calibration
 * from Planar Grids, ICRA 2007
 */

class CataCamera: public Camera
{
public:
    class Parameters: public Camera::Parameters
    {
    public:
        Parameters();
        Parameters(const std::string& cameraName,
                   int w, int h,
                   double xi,
                   double k1, double k2, double p1, double p2,
                   double gamma1, double gamma2, double u0, double v0);

        double& xi(void);
        double& k1(void);
        double& k2(void);
        double& p1(void);
        double& p2(void);
        double& gamma1(void);
        double& gamma2(void);
        double& u0(void);
        double& v0(void);

        double xi(void) const;
        double k1(void) const;
        double k2(void) const;
        double p1(void) const;
        double p2(void) const;
        double gamma1(void) const;
        double gamma2(void) const;
        double u0(void) const;
        double v0(void) const;

        bool readFromYamlFile(const std::string& filename);
        void writeToYamlFile(const std::string& filename) const;

        Parameters& operator=(const Parameters& other);
        friend std::ostream& operator<< (std::ostream& out, const Parameters& params);

    private:
        double m_xi;
        double m_k1;
        double m_k2;
        double m_p1;
        double m_p2;
        double m_gamma1;
        double m_gamma2;
        double m_u0;
        double m_v0;
    };

    CataCamera();

    /**
    * \brief Constructor from the projection model parameters
    */
    CataCamera(const std::string& cameraName,
               int imageWidth, int imageHeight,
               double xi, double k1, double k2, double p1, double p2,
               double gamma1, double gamma2, double u0, double v0);
    /**
    * \brief Constructor from the projection model parameters
    */
    CataCamera(const Parameters& params);

    Camera::ModelType modelType(void) const;
    const std::string& cameraName(void) const;
    int imageWidth(void) const;
    int imageHeight(void) const;

    void estimateIntrinsics(const cv::Size& boardSize,
                            const std::vector< std::vector<cv::Point3f> >& objectPoints,
                            const std::vector< std::vector<cv::Point2f> >& imagePoints);

    // Lift points from the image plane to the sphere
    void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P

    // Lift points from the image plane to the projective space
    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P

    // Projects 3D points to the image plane (Pi function)
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const;
    //%output p

    // Projects 3D points to the image plane (Pi function)
    // and calculates jacobian
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
                      Eigen::Matrix<double,2,3>& J) const;
    //%output p
    //%output J

    void undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const;
    //%output p

    template <typename T>
    static void spaceToPlane(const T* const params,
                             const T* const q, const T* const t,
                             const Eigen::Matrix<T, 3, 1>& P,
                             Eigen::Matrix<T, 2, 1>& p);

    void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const;
    void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u,
                    Eigen::Matrix2d& J) const;

    void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale = 1.0) const;
    cv::Mat initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                    float fx = -1.0f, float fy = -1.0f,
                                    cv::Size imageSize = cv::Size(0, 0),
                                    float cx = -1.0f, float cy = -1.0f,
                                    cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const;

    int parameterCount(void) const;

    const Parameters& getParameters(void) const;
    void setParameters(const Parameters& parameters);

    void readParameters(const std::vector<double>& parameterVec);
    void writeParameters(std::vector<double>& parameterVec) const;

    void writeParametersToYamlFile(const std::string& filename) const;

    std::string parametersToString(void) const;

private:
    Parameters mParameters;

    double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
    bool m_noDistortion;
};

typedef std::shared_ptr<CataCamera> CataCameraPtr;
typedef std::shared_ptr<const CataCamera> CataCameraConstPtr;

}

#endif
