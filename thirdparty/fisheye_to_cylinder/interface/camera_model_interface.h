//
// Created by chenghe on 8/3/22.
//

#ifndef CAMERA_MODEL_CAMERA_MODEL_INTERFACE_H
#define CAMERA_MODEL_CAMERA_MODEL_INTERFACE_H

#include <vector>
#include <array>
#include <cmath>
#include <memory>

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>
namespace haomo_cam
{
    enum CameraModelType
    {
        FISHEYE = 0,
        CYLINDER,
    };

    // 相机参数
    struct CameraParameters
    {
        CameraModelType type{CameraModelType::FISHEYE};
        std::int32_t camera_height{0};
        std::int32_t camera_width{0};
        std::float_t fx;
        std::float_t fy;
        std::float_t cx;
        std::float_t cy;
        std::array<std::double_t, 3> t_body_cam{0, 0, 0};   //x, y, z
        std::array<std::double_t, 4> q_body_cam{1, 0, 0, 0};//w, x, y, z
        std::vector<std::float_t> distortion_params;        //4 for fisheye
    };

    struct CylinderParameters
    {
        std::int32_t camera_height{768};
        std::int32_t camera_width{1024};
        std::float_t horizontal_fov_deg{180}; //水平FOV
        std::float_t horizontal_fov_shift{0}; //水平主点偏移角度
        std::float_t vertical_fov_upper{32.0646}; //垂直向上FOV
        std::float_t vertical_fov_lower{59.9672}; //垂直向下FOV
        bool         automatic_camera_height{false}; //自适应柱面图形高度，false则使用相机高度
        bool         force_set_cylinder_camera_pitch_roll_zero{true};//强制设置柱面虚拟相机roll, pitch角度为0
    };

    class Camera
    {
    public:
        typedef std::shared_ptr<Camera> Ptr;
        typedef std::shared_ptr<const Camera> ConstPtr;

        // 创建一个鱼眼相机
        static Ptr create(const CameraParameters& camera_params); 

        // 创建一个柱面虚拟相机
        static Ptr create_cylinder_camera(const ConstPtr& camera_ptr,
                                          const CylinderParameters& cylinder_params); 

        // 获取相机参数
        virtual CameraParameters get_camera_parameters() const = 0; 

        // 获取外参
        virtual Eigen::Isometry3d get_camera_to_body_transformation() const = 0;

        virtual Eigen::Isometry3d get_body_to_camera_transformation() const = 0;

        // 将图像上的点投影到单位圆上
        virtual Eigen::Vector3d   image_to_unit_ray(const Eigen::Vector2d& pt_image) const = 0;

        // 将图像上的点投影到单位平面上
        virtual Eigen::Vector3d   image_to_unit_plane(const Eigen::Vector2d& pt_image) const = 0;

        // 相机坐标系上的点投影到图像上
        virtual Eigen::Vector2d   camera_to_image(const Eigen::Vector3d& pt_cam) const= 0;

        // 相机坐标系上的点转换到车体坐标系
        virtual Eigen::Vector3d   body_to_camera(const Eigen::Vector3d& pt_body) const = 0;

        // 车体坐标系上的点转换到相机坐标系
        virtual Eigen::Vector3d   camera_to_body(const Eigen::Vector3d& pt_body) const = 0;

        virtual ~Camera() = default;
    };

    class Dewarper
    {
    public:
        typedef std::shared_ptr<Dewarper> Ptr;
        typedef std::shared_ptr<const Dewarper> ConstPtr;

        static Ptr create(const Camera::ConstPtr& fisheye_camera_ptr,
                          const Camera::ConstPtr& cylinder_camera_ptr);

        // 将鱼眼图像转到柱面图
        virtual cv::Mat dewarp(const cv::Mat& image) const = 0;
    };
}


#endif //CAMERA_MODEL_CAMERA_MODEL_INTERFACE_H
