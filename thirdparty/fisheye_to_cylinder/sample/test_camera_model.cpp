//
// Created by chenghe on 8/3/22.
//
#include <iostream>
#include "camera_model_interface.h"

Eigen::Matrix3d construct_homograph(const Eigen::Matrix3d& R_bc, const Eigen::Vector3d& t_bc,
                                    double distance_vehicle_rear_center_to_car_mid,
                                    double wheel_diameter)
{
    Eigen::Vector3d t_bc1 = t_bc + Eigen::Vector3d(-distance_vehicle_rear_center_to_car_mid, 0, 0);
    Eigen::Matrix3d R_cb = R_bc.transpose();
    Eigen::Vector3d t_cb = -R_cb * t_bc1;
    Eigen::Matrix3d H_cb = R_cb - t_cb * Eigen::Vector3d(0, 0, 1.0).transpose() / (0.5 * wheel_diameter);

    Eigen::Matrix3d H = H_cb.inverse();
    H/=H(2, 2);

    return H;
}

cv::Mat ipm_forward(const haomo_cam::Camera::Ptr& fisheye_camera_ptr,
                    cv::Mat& image,
                    int stitch_size = 1200,
                    double pixel_size = 0.02,
                    double distance_vehicle_rear_center_to_car_mid = 1.5,
                    double wheel_diameter = 0.786)
{
    Eigen::Matrix3d H = construct_homograph(fisheye_camera_ptr->get_camera_to_body_transformation().rotation(),
                                            fisheye_camera_ptr->get_camera_to_body_transformation().translation(),
                                            distance_vehicle_rear_center_to_car_mid,
                                            wheel_diameter);

    cv::Mat out(stitch_size, stitch_size, CV_8UC3);
    out.setTo(0);
    for(int x = 0; x < image.cols; ++x)
    {
        for(int y = 0; y < image.rows; ++y)
        {
            Eigen::Vector3d pt_plane = fisheye_camera_ptr->image_to_unit_plane(Eigen::Vector2d{x, y});
            Eigen::Vector3d pt_stitch = H * pt_plane;
            pt_stitch /= pt_stitch.z();
            pt_stitch *= (-wheel_diameter*0.5);
            Eigen::Vector3d pt_cam = fisheye_camera_ptr->body_to_camera(pt_stitch);
            if(pt_cam.z() < 0.0)
            {
                continue;
            }
            int stitch_y = std::round(-pt_stitch.x() / pixel_size + stitch_size/2.0);
            int stitch_x = std::round(-pt_stitch.y() / pixel_size + stitch_size/2.0);

            if(stitch_x >= 0 && stitch_x < stitch_size && stitch_y >= 0 && stitch_y < stitch_size)
            {
                out.at<cv::Vec3b>(stitch_y, stitch_x) = image.at<cv::Vec3b>(y, x);
            }
        }
    }

    return out;
}

cv::Mat ipm_backward(const haomo_cam::Camera::Ptr& fisheye_camera_ptr,
                     cv::Mat& image,
                     int stitch_size = 1200,
                     double pixel_size = 0.02,
                     double distance_vehicle_rear_center_to_car_mid = 1.5,
                     double wheel_diameter = 0.786)
{
    double cx = stitch_size/2.0 ;
    double cy = stitch_size/2.0 + distance_vehicle_rear_center_to_car_mid/pixel_size;
    cv::Mat out(stitch_size, stitch_size, CV_8UC3);
    out.setTo(0);
    for(int x = 0; x < stitch_size; ++x)
    {
        for(int y = 0; y < stitch_size; ++y)
        {
            Eigen::Vector3d pt_body{-(y - cy)*pixel_size, -(x - cx)*pixel_size, -wheel_diameter * 0.5};
            Eigen::Vector3d pt_cam = fisheye_camera_ptr->body_to_camera(pt_body);
            Eigen::Vector2d pt_image = fisheye_camera_ptr->camera_to_image(pt_cam);
            int cam_x = std::round(pt_image.x());
            int cam_y = std::round(pt_image.y());

            if(cam_x >= 0 && cam_x < image.cols && cam_y >= 0 && cam_y < image.rows)
            {
                out.at<cv::Vec3b>(y, x) = image.at<cv::Vec3b>(cam_y, cam_x);
            }
        }
    }

    return out;
}

int main()
{
    haomo_cam::CameraParameters fisheye_camera_params;
    fisheye_camera_params.fx = 469.435394;
    fisheye_camera_params.fy = 469.852112;
    fisheye_camera_params.cx = 967.581665;
    fisheye_camera_params.cy = 538.946777;
    fisheye_camera_params.distortion_params =
            std::vector<float>{0.0640982, -0.0122619588, 0.00125413574, -0.000395158};
    fisheye_camera_params.camera_width = 1920;
    fisheye_camera_params.camera_height = 1080;
    fisheye_camera_params.q_body_cam =
            std::array<std::double_t, 4>{-0.0140862009, -0.0182368234, 0.821001172, -0.570461273};
    fisheye_camera_params.t_body_cam = std::array<std::double_t, 3>{2.14122534, -0.929897547, 0.640055537};

    haomo_cam::CylinderParameters cylinder_parameters;
    cylinder_parameters.camera_width  = 1024;
    cylinder_parameters.camera_height = 768;
    cylinder_parameters.horizontal_fov_deg = 180;
    cylinder_parameters.vertical_fov_upper = 32.0646;
    cylinder_parameters.vertical_fov_lower = 59.9672;
    cylinder_parameters.automatic_camera_height = false;

    auto fisheye_cam_ptr = haomo_cam::Camera::create(fisheye_camera_params);

    auto cylinder_cam_ptr = haomo_cam::Camera::create_cylinder_camera(fisheye_cam_ptr, cylinder_parameters);

    auto dewarper_ptr = haomo_cam::Dewarper::create(fisheye_cam_ptr, cylinder_cam_ptr);

    cv::Mat image = cv::imread("/home/data/CLionProjects/fisheye_to_cylinder/resource/V71C007/fisheye_images/right_fisheye_camera.jpg");

    auto forward = ipm_forward(fisheye_cam_ptr, image, 600, 0.04);
    auto backward = ipm_backward(fisheye_cam_ptr, image, 600, 0.04);

    cv::Mat hout;
    cv::hconcat(forward, backward, hout);

    for(int i = 0; i < 15; ++i)
    {
        cv::line(hout, cv::Point(0, i*40), cv::Point(1199, i*40), cv::Scalar(255, 255, 255), 1, 16);
    }

    cv::namedWindow("ipm_h", cv::WINDOW_NORMAL);
    cv::imshow("ipm_h", hout);

    cv::Mat vout;
    cv::vconcat(forward, backward, vout);

    for(int i = 0; i < 15; ++i)
    {
        cv::line(vout, cv::Point(i*40, 0), cv::Point(i*40, 1199), cv::Scalar(255, 255, 255), 1, 16);
    }

    cv::namedWindow("ipm_v", cv::WINDOW_NORMAL);
    cv::imshow("ipm_v", vout);
    cv::waitKey(0);

    cv::imwrite("/home/chenghe/v_ipm.jpg", vout);
    cv::imwrite("/home/chenghe/h_ipm.jpg", hout);
    return 0;
}