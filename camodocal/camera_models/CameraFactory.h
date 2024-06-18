#ifndef CAMERAFACTORY_H
#define CAMERAFACTORY_H

#include <memory>
#include <opencv2/core/core.hpp>
#include <iostream>

#include "../camera_models/Camera.h"

namespace camodocal
{

class CameraFactory
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CameraFactory();

    static std::shared_ptr<CameraFactory> instance(void);

    CameraPtr generateCamera(Camera::ModelType modelType,
                             const std::string& cameraName,
                             cv::Size imageSize) const;

    CameraPtr generateCameraFromYamlFile(const std::string& filename);

private:
    static std::shared_ptr<CameraFactory> m_instance;
};

}

#endif
