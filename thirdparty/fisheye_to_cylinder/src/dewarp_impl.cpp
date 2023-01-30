//
// Created by chenghe on 8/3/22.
//
#include "dewarp_impl.h"

namespace haomo_cam
{

    bool DewarperImpl::init(const Camera::ConstPtr& fisheye_camera_ptr,
                            const Camera::ConstPtr& cylinder_camera_ptr)
    {
        if (fisheye_camera_ptr == nullptr || fisheye_camera_ptr == nullptr) {
            printf("%s[%d] Param error !\n", __FUNCTION__, __LINE__);
            return false;
        }

        const int in_width{fisheye_camera_ptr->get_camera_parameters().camera_width};
        const int in_height{fisheye_camera_ptr->get_camera_parameters().camera_height};

        const int out_width{cylinder_camera_ptr->get_camera_parameters().camera_width};
        const int out_height{cylinder_camera_ptr->get_camera_parameters().camera_height};

        // param check
        if ((in_width <= 1) || (in_height <= 1) || (out_width <= 1) || (out_height <= 1))
        {
            printf("%s[%d] camera param error !\n", __FUNCTION__, __LINE__);
            return false;
        }

        warp_map_xy_.create(out_height, out_width, CV_32FC2);

        for (int y{0}; y < out_height; y++) {
            for (int x{0}; x < out_width; x++) {
                Eigen::Vector3d unit_p3d_outcam{cylinder_camera_ptr->image_to_unit_ray(Eigen::Vector2d(x, y))};
                Eigen::Vector3d unit_p3d_incam{
                        fisheye_camera_ptr->body_to_camera(cylinder_camera_ptr->camera_to_body(unit_p3d_outcam))};
                Eigen::Vector2d pixel_incam{fisheye_camera_ptr->camera_to_image(unit_p3d_incam)};

                if (!((pixel_incam.x() >= 0.0) && (pixel_incam.x() <= (in_width - 1.0)) && (pixel_incam.y() >= 0.0) &&
                      (pixel_incam.y() <= (in_height - 1.0)))) {
                    pixel_incam.x() = -10.0;
                    pixel_incam.y() = -10.0;
                }

                warp_map_xy_.at<cv::Vec2f>(y, x) = cv::Vec2f(pixel_incam.x(), pixel_incam.y());
            }
        }

        return true;
    }

    cv::Mat DewarperImpl::dewarp(const cv::Mat& image) const
    {
        cv::Mat dst;
        cv::remap(image, dst, warp_map_xy_, cv::Mat(), cv::InterpolationFlags::INTER_LINEAR);
        return dst;
    }

    Dewarper::Ptr Dewarper::create(const Camera::ConstPtr& fisheye_camera_ptr,
                                   const Camera::ConstPtr& cylinder_camera_ptr)
    {
        auto ptr = std::make_shared<DewarperImpl>();
        if(!ptr->init(fisheye_camera_ptr, cylinder_camera_ptr))
        {
            return nullptr;
        }

        return ptr;
    }
}