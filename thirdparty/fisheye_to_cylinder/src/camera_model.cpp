//
// Created by chenghe on 8/3/22.
//
#include "camera_model.h"
#include "cylinder_camera_model.h"
#include "fisheye_camera_model.h"
namespace haomo_cam
{
    CameraParameters BaseCamera::get_camera_parameters() const
    {
        return camera_parameters_;
    }

    Eigen::Isometry3d BaseCamera::get_camera_to_body_transformation() const
    {
        return T_body_cam_;
    }

    inline Eigen::Isometry3d BaseCamera::get_body_to_camera_transformation() const
    {
    return T_cam_body_;
    }

    Eigen::Vector3d BaseCamera::body_to_camera(const Eigen::Vector3d& pt_body) const
    {
        Eigen::Vector3d pt_cam = T_cam_body_ * pt_body;

        return pt_cam;
    }

    Eigen::Vector3d BaseCamera::camera_to_body(const Eigen::Vector3d& pt_cam) const
    {
        Eigen::Vector3d pt_body = T_body_cam_ * pt_cam;

        return pt_body;
    }

    void BaseCamera::set_camera_parameters(const CameraParameters& camera_parameters)
    {
        camera_parameters_ = camera_parameters;
        Eigen::Quaterniond q{camera_parameters.q_body_cam[0],
                             camera_parameters.q_body_cam[1],
                             camera_parameters.q_body_cam[2],
                             camera_parameters.q_body_cam[3]};

        Eigen::Vector3d    t{camera_parameters.t_body_cam[0],
                             camera_parameters.t_body_cam[1],
                             camera_parameters.t_body_cam[2]};

        T_body_cam_.setIdentity();
        T_body_cam_.linear()      = q.toRotationMatrix();
        T_body_cam_.translation() = t;

        T_cam_body_ = T_body_cam_.inverse();
    }

    Camera::Ptr Camera::create(const CameraParameters& camera_params)
    {
        auto ptr = std::make_shared<FisheyeCamera>();
        ptr->init(camera_params);

        return ptr;
    }

    Camera::Ptr Camera::create_cylinder_camera(const ConstPtr& camera_ptr,
                                               const CylinderParameters& cylinder_params)
    {
        auto ptr = std::make_shared<CylinderCamera>();
        ptr->init(camera_ptr, cylinder_params);

        return ptr;
    }
}