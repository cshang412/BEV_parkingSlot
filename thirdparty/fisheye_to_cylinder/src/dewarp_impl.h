//
// Created by chenghe on 8/3/22.
//

#ifndef CAMERA_MODEL_DEWARP_IMPL_H
#define CAMERA_MODEL_DEWARP_IMPL_H
#include "camera_model_interface.h"

namespace haomo_cam
{
    class DewarperImpl: public Dewarper
    {
    public:

        bool init(const Camera::ConstPtr& fisheye_camera_ptr,
                  const Camera::ConstPtr& cylinder_camera_ptr);

        cv::Mat dewarp(const cv::Mat& image) const override;

    private:

        cv::Mat warp_map_xy_;
    };
}

#endif //CAMERA_MODEL_DEWARP_IMPL_H
