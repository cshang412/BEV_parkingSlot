//
// Created by chenghe on 8/3/22.
//
#include "fisheye_camera_model.h"
namespace haomo_cam
{
    void FisheyeCamera::init(const CameraParameters& camera_parameters)
    {
        std::float_t f = (camera_parameters.fx+camera_parameters.fy) * 0.5f;
        CameraParameters new_camera_parameter = camera_parameters;
        new_camera_parameter.fx = f;
        new_camera_parameter.fy = f;
        set_camera_parameters(new_camera_parameter);

        max_valid_theta_         = 0.5 * M_PI;
        max_valid_radius_        = calculate_radius_by_theta(max_valid_theta_);
        radius_theta_lut_res_    = 0.125;
        radius_theta_lut_length_ = (static_cast<int>(max_valid_radius_) / radius_theta_lut_res_) + 10;

        generate_LUT(max_valid_theta_, radius_theta_lut_length_, radius_theta_lut_res_);
    }

    Eigen::Vector3d FisheyeCamera::image_to_unit_ray(const Eigen::Vector2d& pt_image) const
    {
        Eigen::Vector3d p3d;
        const double    xd{pt_image.x() - camera_parameters_.cx};
        const double    yd{pt_image.y() - camera_parameters_.cy};
        const double    phi{atan2(yd, xd)};
        double          r2{(xd * xd) + (yd * yd)};
        r2 = (r2 > std::numeric_limits<double>::min()) ? r2 : std::numeric_limits<double>::min();
        const double r{sqrt(r2)}; // -- radius to image center
        const double theta{lookup_theta_by_radius(r)};
        const double r_recalculated{calculate_radius_by_theta(theta)};

        if (theta > 0.0)
        {
            constexpr double sphrad{1.F};
            p3d.z() = sphrad * cos(theta);
            double r_undistort2{(sphrad * sphrad) - (p3d.z() * p3d.z())};
            r_undistort2 =
                    (r_undistort2 > std::numeric_limits<double>::min()) ? r_undistort2 : std::numeric_limits<double>::min();
            const double r_undistort{sqrt(r_undistort2)};
            p3d.x() = r_undistort * cos(phi);
            p3d.y() =
                    r_undistort * sin(phi); // -- compute spherical coordinates based on theta, phi and radius of the sphere
        }
        else
        {
            p3d.x() = (p3d.y() = (p3d.z() = std::numeric_limits<double>::quiet_NaN()));
        }
        return p3d;
    }

    Eigen::Vector3d FisheyeCamera::image_to_unit_plane(const Eigen::Vector2d& pt_image) const
    {
        Eigen::Vector3d p3d(0, 0, 0);
        const double    xd{pt_image.x() - camera_parameters_.cx};
        const double    yd{pt_image.y() - camera_parameters_.cy};
        const double    phi{atan2(yd, xd)};
        double          r2{(xd * xd) + (yd * yd)};
        r2 = (r2 > std::numeric_limits<double>::min()) ? r2 : std::numeric_limits<double>::min();
        const double r{sqrt(r2)}; // -- radius to image center
        const double theta{lookup_theta_by_radius(r)};
        const double r_recalculated{calculate_radius_by_theta(theta)};

        if (theta > 0.0)
        {
            constexpr double sphrad{1.F};
            p3d.z() = sphrad * cos(theta);
            double r_undistort2{(sphrad * sphrad) - (p3d.z() * p3d.z())};
            r_undistort2 =
                    (r_undistort2 > std::numeric_limits<double>::min()) ? r_undistort2 : std::numeric_limits<double>::min();
            const double r_undistort{sqrt(r_undistort2)};
            p3d.x() = r_undistort * cos(phi);
            p3d.y() = r_undistort * sin(phi); // -- compute spherical coordinates based on theta, phi and radius of the sphere

            return Eigen::Vector3d(p3d.x() / p3d.z(), p3d.y() / p3d.z(), 1.0);
        }
        else
        {
            return Eigen::Vector3d(p3d.x() / p3d.z(), p3d.y() / p3d.z(), 1.0);
        }
    }

    Eigen::Vector2d FisheyeCamera::camera_to_image(const Eigen::Vector3d& pt_cam) const
    {
        const double phi2{atan2(pt_cam.y(), pt_cam.x())};
        double       cos_value{pt_cam.z() / pt_cam.norm()};
        cos_value = (cos_value > 1.0) ? 1.0 : ((cos_value < -1.0) ? -1.0 : cos_value);
        const double theta{acos(cos_value)};

        if (theta > max_valid_theta_)
        {
            return Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
        }

        const double theta2{theta * theta};
        const double theta4{theta2 * theta2};
        const double theta6{theta2 * theta4};
        const double theta8{theta4 * theta4};

        const double dr{camera_parameters_.fx * theta *
                        (1 + (camera_parameters_.distortion_params[0] * theta2) +
                         (camera_parameters_.distortion_params[1] * theta4) +
                         (camera_parameters_.distortion_params[2] * theta6) +
                         (camera_parameters_.distortion_params[3] * theta8))};

        if (dr > max_valid_radius_)
        {
            return Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
        }
        else
        {
            return Eigen::Vector2d((dr * cos(phi2)) + camera_parameters_.cx, (dr * sin(phi2)) + camera_parameters_.cy);
        }
    }

    bool FisheyeCamera::generate_LUT(const double max_valid_theta, const int radius_theta_lut_length,
                                     const double radius_theta_lut_res)
    {
        max_valid_theta_         = max_valid_theta;
        max_valid_radius_        = calculate_radius_by_theta(max_valid_theta_);
        radius_theta_lut_res_    = radius_theta_lut_res;
        radius_theta_lut_length_ = radius_theta_lut_length;

        if (radius_theta_lut_length_ <= 0)
        {
            return false;
        }
        // if (radius_theta_lut_ != nullptr) {
        //     delete[](radius_theta_lut_);
        //     radius_theta_lut_ = nullptr;
        // }
        // radius_theta_lut_ = new double[radius_theta_lut_length_];
        radius_theta_lut_.resize(radius_theta_lut_length_);
        for (int i{0}; i < radius_theta_lut_length_; i++)
        {
            const double lut_radius{i * radius_theta_lut_res_};
            radius_theta_lut_[i] = calculate_theta_by_radius(lut_radius);
        }

        return true;
    }

    double FisheyeCamera::calculate_radius_by_theta(const double theta) const
    {
        if (theta < 0.0) {
            return -1.0;
        }
        const double theta2{theta * theta};
        const double theta4{theta2 * theta2};
        const double theta6{theta2 * theta4};
        const double theta8{theta4 * theta4};

        const double dr{camera_parameters_.fx * theta *
                        (1 + (camera_parameters_.distortion_params[0] * theta2) +
                         (camera_parameters_.distortion_params[1] * theta4) +
                         (camera_parameters_.distortion_params[2] * theta6) +
                         (camera_parameters_.distortion_params[3] * theta8))};

        return dr;
    }
    double FisheyeCamera::calculate_theta_by_radius(const double radius) const
    {
        if ((radius > max_valid_radius_) || (radius < 0.0)) {
            return -1;
        }
        double min_val{0.0};
        double max_val{0.5 * M_PI};

        if ((calculate_radius_by_theta(min_val) <= radius) && (calculate_radius_by_theta(max_val) >= radius))
        {
            while ((max_val - min_val) > 0.0001)
            {
                const double mid_val{0.5 * (min_val + max_val)};

                const double midval_radius{calculate_radius_by_theta(mid_val)};
                const double minval_radius{calculate_radius_by_theta(min_val)};
                const double maxval_radius{calculate_radius_by_theta(max_val)};

                if (!((minval_radius <= radius) && (maxval_radius >= radius)))
                {
                    return -1.0;
                }
                else
                {
                    if (midval_radius <= radius)
                    {
                        min_val = mid_val;
                    }
                    else
                    {
                        max_val = mid_val;
                    }
                }
            }
        }
        else
        {
            return -1.0;
        }

        const double minval_radius{calculate_radius_by_theta(min_val)};
        const double maxval_radius{calculate_radius_by_theta(max_val)};
        const double dt{(radius - minval_radius) / (maxval_radius - minval_radius)};

        return ((1.0 - dt) * min_val) + ((dt)*max_val);
    }
    double FisheyeCamera::lookup_theta_by_radius(const double radius) const
    {
        const double radius_idx_double{radius / radius_theta_lut_res_};
        const int    radius_idx_int{static_cast<int>(radius_idx_double)};
        const double interp_dt{radius_idx_double - radius_idx_int};

        if ((radius_idx_int >= 0) && (radius_idx_int < (radius_theta_lut_length_ - 1)))
        {
            const double theta0{radius_theta_lut_[radius_idx_int]};
            const double theta1{radius_theta_lut_[radius_idx_int + 1]};
            if ((theta0 >= 0.0) && (theta1 >= 0.0))
            {
                return (theta0 * (1 - interp_dt)) + (theta1 * interp_dt);
            }
            else
            {
                return -1.0;
            }
        }
        else
        {
            return -1.0;
        }
    }

}