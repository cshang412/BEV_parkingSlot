// Created by chenghe on 11/12/22.
//

#ifndef PSD_MAPPING_HARDCONF_H
#define PSD_MAPPING_HARDCONF_H


#include "stitch_interface.h"
#include "camera_model_interface.h"
#include "json/json.h"

using namespace psd_mapping;

namespace psd_mapping
{
    class HardwareConfig
    {
    public:

        bool load_from_json(const std::string& json_path)
        {
            std::vector<std::string> cam_name_vec{"front_fisheye_camera",
                                                "right_fisheye_camera",
                                                "rear_fisheye_camera",
                                                "left_fisheye_camera"};

            auto root = Util::load_json(json_path);

            car_param.car_length_ = 5.0;
            car_param.car_width_ = 1.891;
            car_param.rearshaft2carcenter_ = 1.5;
            car_param.wheel_diameter = 0.786;

            stitch_param.use_car_rect_  = true;
            stitch_param.image_width_  = 1200;
            stitch_param.image_height_  = 1200;
            stitch_param.pix_resolution_ = 0.02;
            stitch_param.extend_pix_ = 0;
            stitch_param.lean_angle_ = 5.0;
            stitch_param.use_car_rect_ = true;

            for(int ci = 0; ci < 4; ++ci)
            {
                camera_param_vec.push_back(load_camera_param(root[cam_name_vec[ci]]));
                camera_ptr_vec.push_back(create_camera_ptr(camera_param_vec[ci]));
                Hs.push_back(construct_homograph(camera_param_vec[ci], car_param.rearshaft2carcenter_, car_param.wheel_diameter));
            }

            void * void_camera_params[4];
            for (int cid = 0; cid < 4; cid++)
            {
                void_camera_params[cid] = (void *)&(camera_param_vec[cid]);
            }

            haomo::stitch::DataType process_data_type = haomo::stitch::DataType::DT_YUYV;
            haomo::stitch::LutInfo lut_info;
            haomo::stitch::IPM_generate_lut(void_camera_params, car_param, &stitch_param, &lut_info, process_data_type);
            blank_mask.create(1200, 1200, CV_8UC1);
            haomo::stitch::BlankArea blank_area;
            haomo::stitch::IPM_get_blank_mask(blank_area, &blank_mask.data);

            return true;
        }

        haomo::stitch::CameraParam load_camera_param(const Json::Value& value) const
        {
            haomo::stitch::CameraParam cam_pam;
            cam_pam.camera_qvec_[0] = value["attitude"]["w"].asDouble();
            cam_pam.camera_qvec_[1] = value["attitude"]["x"].asDouble();
            cam_pam.camera_qvec_[2] = value["attitude"]["y"].asDouble();
            cam_pam.camera_qvec_[3] = value["attitude"]["z"].asDouble();

            cam_pam.camera_tvec_[0] = value["translation"]["x"].asDouble();
            cam_pam.camera_tvec_[1] = value["translation"]["y"].asDouble();
            cam_pam.camera_tvec_[2] = value["translation"]["z"].asDouble();

            cam_pam.fx_ = value["fx"].asDouble();
            cam_pam.fy_ = value["fy"].asDouble();
            cam_pam.cx_ = value["cx"].asDouble();
            cam_pam.cy_ = value["cy"].asDouble();

            cam_pam.k2_ = value["distortion"][0].asDouble();
            cam_pam.k3_ = value["distortion"][1].asDouble();
            cam_pam.k4_ = value["distortion"][2].asDouble();
            cam_pam.k5_ = value["distortion"][3].asDouble();
            cam_pam.image_height_ = value["height"].asInt();
            cam_pam.image_width_  = value["width"].asInt();

            return cam_pam;
        }

        static haomo_cam::Camera::Ptr create_camera_ptr(const haomo::stitch::CameraParam& camera_param)
        {
            haomo_cam::CameraParameters fisheye_camera_params;
            fisheye_camera_params.fx = camera_param.fx_;
            fisheye_camera_params.fy = camera_param.fy_;
            fisheye_camera_params.cx = camera_param.cx_;
            fisheye_camera_params.cy = camera_param.cy_;
            fisheye_camera_params.distortion_params =
                    std::vector<float>{(float)camera_param.k2_,
                                    (float)camera_param.k3_,
                                    (float)camera_param.k4_,
                                    (float)camera_param.k5_};
            fisheye_camera_params.camera_width = camera_param.image_width_;;
            fisheye_camera_params.camera_height = camera_param.image_height_;
            fisheye_camera_params.q_body_cam =
                    std::array<std::double_t, 4>{camera_param.camera_qvec_[0],
                                                camera_param.camera_qvec_[1],
                                                camera_param.camera_qvec_[2],
                                                camera_param.camera_qvec_[3]};
            fisheye_camera_params.t_body_cam = std::array<std::double_t, 3>{camera_param.camera_tvec_[0],
                                                                            camera_param.camera_tvec_[1],
                                                                            camera_param.camera_tvec_[2]};

            auto fisheye_cam_ptr = haomo_cam::Camera::create(fisheye_camera_params);

            return fisheye_cam_ptr;
        }

        static Eigen::Matrix3d construct_homograph(const haomo::stitch::CameraParam& cam_param,
                                            double distance_vehicle_rear_center_to_car_mid,
                                            double wheel_diameter)
        {
            Eigen::Quaterniond q_bc(cam_param.camera_qvec_[0],
                                cam_param.camera_qvec_[1],
                                cam_param.camera_qvec_[2],
                                cam_param.camera_qvec_[3]);
            Eigen::Vector3d t_bc(cam_param.camera_tvec_[0],
                            cam_param.camera_tvec_[1],
                            cam_param.camera_tvec_[2]);

            Eigen::Vector3d t_bc1 = t_bc + Eigen::Vector3d(-distance_vehicle_rear_center_to_car_mid, 0, 0);
            Eigen::Matrix3d R_cb = q_bc.toRotationMatrix().transpose();
            Eigen::Vector3d t_cb = -R_cb * t_bc1;
            Eigen::Matrix3d H_cb = R_cb - t_cb * Eigen::Vector3d(0, 0, 1.0).transpose() / (0.5 * wheel_diameter);

            Eigen::Matrix3d H = H_cb.inverse();
            H/=H(2, 2);

            return H;
        }

        cv::Mat blank_mask;
        std::vector<Eigen::Matrix3d> Hs;
        haomo::stitch::StitchParam stitch_param;
        haomo::stitch::CarParam    car_param;
        std::vector<haomo::stitch::CameraParam> camera_param_vec;
        std::vector<haomo_cam::Camera::Ptr>     camera_ptr_vec;
    };
}

#endif 