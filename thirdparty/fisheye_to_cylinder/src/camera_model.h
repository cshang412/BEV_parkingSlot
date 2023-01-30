//
// Created by chenghe on 8/3/22.
//

#ifndef CAMERA_MODEL_CAMERA_MODEL_H
#define CAMERA_MODEL_CAMERA_MODEL_H

#include "camera_model_interface.h"

namespace haomo_cam
{
    class BaseCamera: public Camera
    {
    public:
        CameraParameters get_camera_parameters() const override;

        Eigen::Isometry3d get_camera_to_body_transformation() const override;

        inline Eigen::Isometry3d get_body_to_camera_transformation() const override;

        Eigen::Vector3d   image_to_unit_ray(const Eigen::Vector2d& pt_image) const override = 0;

        Eigen::Vector3d   image_to_unit_plane(const Eigen::Vector2d& pt_image) const override = 0;

        Eigen::Vector2d   camera_to_image(const Eigen::Vector3d& pt_cam) const override = 0;

        Eigen::Vector3d body_to_camera(const Eigen::Vector3d& pt_body) const override;

        Eigen::Vector3d   camera_to_body(const Eigen::Vector3d& pt_body) const override;

        ~BaseCamera() override = default;

    protected:

        void set_camera_parameters(const CameraParameters& camera_parameters);

        CameraParameters camera_parameters_;
        Eigen::Isometry3d T_body_cam_;
        Eigen::Isometry3d T_cam_body_;
    };




}

#endif //CAMERA_MODEL_CAMERA_MODEL_H
