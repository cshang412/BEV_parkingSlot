//
// Created by chenghe on 8/3/22.
//

#ifndef CAMERA_MODEL_FISHEYE_CAMERA_MODEL_H
#define CAMERA_MODEL_FISHEYE_CAMERA_MODEL_H
#include "camera_model.h"
namespace haomo_cam
{
    class FisheyeCamera: public BaseCamera
    {
    public:

        void init(const CameraParameters& camera_parameters);

        Eigen::Vector3d   image_to_unit_ray(const Eigen::Vector2d& pt_image) const override;

        Eigen::Vector3d   image_to_unit_plane(const Eigen::Vector2d& pt_image) const override;

        Eigen::Vector2d   camera_to_image(const Eigen::Vector3d& pt_cam) const override;

    private:
    public:
        bool generate_LUT(const double max_valid_theta, const int radius_theta_lut_length,
                          const double radius_theta_lut_res);

    private:
        double calculate_radius_by_theta(const double theta) const;

        double calculate_theta_by_radius(const double radius) const;

        double lookup_theta_by_radius(const double radius) const;

        double              radius_theta_lut_res_;
        int                 radius_theta_lut_length_;
        double              max_valid_radius_;
        double              max_valid_theta_;
        std::vector<double> radius_theta_lut_;
    };
}

#endif //CAMERA_MODEL_FISHEYE_CAMERA_MODEL_H
