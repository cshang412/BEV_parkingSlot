//
// Created by chenghe on 8/3/22.
//
#include "cylinder_camera_model.h"

namespace haomo_cam
{
    namespace internal
    {
        class Math
        {
        public:
            static double regularize_angle_tough(double angle) noexcept
            {
                const int angle_round{static_cast<int>(angle / (2 * M_PI))};
                angle = angle - (angle_round * (2 * M_PI));
                if (angle > M_PI) {
                    angle -= 2 * M_PI;
                } else if (angle < -M_PI) {
                    angle += 2 * M_PI;
                } else {
                    angle = angle;
                }
                return angle;
            }

            static double extract_yaw_from_rot_mat(const Eigen::Matrix3d &rot_mat)
            {
                Eigen::Vector3d eular{rot_mat.eulerAngles(2, 1, 0)};
                if ((std::abs(eular(1)) + std::abs(eular(2))) > (0.8 * M_PI))
                {
                    return regularize_angle_tough(eular(0) + M_PI);
                }
                else
                {
                    return regularize_angle_tough(eular(0));
                }
            }
        };
    }

    void CylinderCamera::init(const ConstPtr& origin_camera_ptr,
                              const CylinderParameters& cylinder_parameters)
    {
        cylinder_parameters_ = cylinder_parameters;

        auto origin_camera_parameters = origin_camera_ptr->get_camera_parameters();
        CameraParameters new_camera_parameters = origin_camera_ptr->get_camera_parameters();
        if(cylinder_parameters.force_set_cylinder_camera_pitch_roll_zero)
        {
            Eigen::Quaterniond q{new_camera_parameters.q_body_cam[0],
                                 new_camera_parameters.q_body_cam[1],
                                 new_camera_parameters.q_body_cam[2],
                                 new_camera_parameters.q_body_cam[3]};

            Eigen::Vector3d    t{new_camera_parameters.t_body_cam[0],
                                 new_camera_parameters.t_body_cam[1],
                                 new_camera_parameters.t_body_cam[2]};
            T_body_cam_.setIdentity();
            T_body_cam_.linear()      = q.toRotationMatrix();
            T_body_cam_.translation() = t;

            // -- calculate virtual upright extrinsic
            Eigen::Matrix3d camhlu_to_cam_R;
            camhlu_to_cam_R << 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0;
            const Eigen::Matrix3d camhlu_to_body_R{T_body_cam_.linear() * camhlu_to_cam_R};

            double          yaw{internal::Math::extract_yaw_from_rot_mat(camhlu_to_body_R)};
            Eigen::Matrix3d upright_ext_camhlu_body_R{Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()};
            const Eigen::Matrix3d upright_ext_cam_body_R{upright_ext_camhlu_body_R * camhlu_to_cam_R.inverse()};

            Eigen::Isometry3d upright_ext_cam_body{Eigen::Isometry3d::Identity()};
            upright_ext_cam_body.linear()      = upright_ext_cam_body_R;
            upright_ext_cam_body.translation() = T_body_cam_.translation();

            Eigen::Quaterniond new_q{upright_ext_cam_body.linear()};

            new_camera_parameters.q_body_cam[0] = new_q.w();
            new_camera_parameters.q_body_cam[1] = new_q.x();
            new_camera_parameters.q_body_cam[2] = new_q.y();
            new_camera_parameters.q_body_cam[3] = new_q.z();

            new_camera_parameters.t_body_cam[0] = upright_ext_cam_body.translation().x();
            new_camera_parameters.t_body_cam[1] = upright_ext_cam_body.translation().y();
            new_camera_parameters.t_body_cam[2] = upright_ext_cam_body.translation().z();
        }


        new_camera_parameters.distortion_params.clear();
        new_camera_parameters.camera_height = cylinder_parameters.camera_height;
        new_camera_parameters.camera_width  = cylinder_parameters.camera_width;
        new_camera_parameters.type = CameraModelType::CYLINDER;

        static const std::double_t deg_to_rad = M_PI / 180.0;

        new_camera_parameters.fx = static_cast<std::float_t>(new_camera_parameters.camera_width) /
                                   (cylinder_parameters.horizontal_fov_deg * deg_to_rad);
        if(cylinder_parameters_.automatic_camera_height)
        {
            new_camera_parameters.fy = new_camera_parameters.fx;
        }
        else
        {
            new_camera_parameters.fy = new_camera_parameters.camera_height/
                                       (std::tan(cylinder_parameters.vertical_fov_upper * deg_to_rad) +
                                        std::tan(cylinder_parameters.vertical_fov_lower * deg_to_rad));
        }


        const std::double_t upper_height{std::tan(cylinder_parameters.vertical_fov_upper * deg_to_rad) * new_camera_parameters.fy};
        if(cylinder_parameters_.automatic_camera_height)
        {
            const std::double_t lower_height{std::tan(cylinder_parameters.vertical_fov_lower * deg_to_rad) * new_camera_parameters.fy};
            new_camera_parameters.camera_height = static_cast<std::int32_t>(lower_height + upper_height);
        }

        const std::double_t hfov_shift_rad = cylinder_parameters.horizontal_fov_shift * deg_to_rad;
        new_camera_parameters.cx = std::round(new_camera_parameters.camera_width +
                                              hfov_shift_rad * new_camera_parameters.fx * 2.0) * 0.5;
        new_camera_parameters.cy = std::round(upper_height * 2.0) * 0.5;


        set_camera_parameters(new_camera_parameters);
    }

    Eigen::Vector3d CylinderCamera::image_to_unit_ray(const Eigen::Vector2d& pt_image) const
    {
        const double xd{pt_image.x() - camera_parameters_.cx};
        const double yd{pt_image.y() - camera_parameters_.cy};
        const double theta_x{xd / camera_parameters_.fx};
        const double yr{yd / camera_parameters_.fy};
        const double xr{sin(theta_x)};
        const double zr{cos(theta_x)};
        double       distance_2{(xr * xr) + (yr * yr) + (zr * zr)};
        distance_2 = (distance_2 > std::numeric_limits<double>::min()) ? distance_2 : std::numeric_limits<double>::min();
        const double length_inv{1.0 / sqrt(distance_2)};
        return Eigen::Vector3d(length_inv * xr, length_inv * yr, length_inv * zr);
    }

    Eigen::Vector3d CylinderCamera::image_to_unit_plane(const Eigen::Vector2d& pt_image) const
    {
        const double xd{pt_image.x() - camera_parameters_.cx};
        const double yd{pt_image.y() - camera_parameters_.cy};
        const double theta_x{xd / camera_parameters_.fx};
        const double yr{yd / camera_parameters_.fy};
        const double xr{sin(theta_x)};
        const double zr{cos(theta_x)};
        const double z_inv{1.0 / zr};
        return Eigen::Vector3d(z_inv * xr, z_inv * yr, 1.0);
    }

    Eigen::Vector2d CylinderCamera::camera_to_image(const Eigen::Vector3d& pt_cam) const
    {
        const double theta_x{atan(pt_cam.x() / pt_cam.z())};
        double       distance_2{(pt_cam.x() * pt_cam.x()) + (pt_cam.z() * pt_cam.z())};
        distance_2 = (distance_2 > std::numeric_limits<double>::min()) ? distance_2 : std::numeric_limits<double>::min();
        const double yr{pt_cam.y() / sqrt(distance_2)};
        return Eigen::Vector2d((camera_parameters_.fx * theta_x) + camera_parameters_.cx,
                               (camera_parameters_.fy * yr) + camera_parameters_.cy);
    }
}