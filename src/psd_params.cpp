#include "psd_params.h"

namespace psd_mapping
{
    void cameraParam::load_cameraparam(const std::string& path){

        auto root = Util::load_json(path);
        auto front_fisheye_camera = root["front_fisheye_camera"];
        auto right_fisheye_camera = root["right_fisheye_camera"];
        auto rear_fisheye_camera = root["rear_fisheye_camera"];
        auto left_fisheye_camera = root["left_fisheye_camera"];

        Eigen::Quaterniond quatuam1{front_fisheye_camera["attitude"]["w"].asDouble(),
                                    front_fisheye_camera["attitude"]["x"].asDouble(),
                                    front_fisheye_camera["attitude"]["y"].asDouble(),
                                    front_fisheye_camera["attitude"]["z"].asDouble()};
        Eigen::Quaterniond quatuam2{right_fisheye_camera["attitude"]["w"].asDouble(),
                                    right_fisheye_camera["attitude"]["x"].asDouble(),
                                    right_fisheye_camera["attitude"]["y"].asDouble(),
                                    right_fisheye_camera["attitude"]["z"].asDouble()};
        Eigen::Quaterniond quatuam3{rear_fisheye_camera["attitude"]["w"].asDouble(),
                                    rear_fisheye_camera["attitude"]["x"].asDouble(),
                                    rear_fisheye_camera["attitude"]["y"].asDouble(),
                                    rear_fisheye_camera["attitude"]["z"].asDouble()};
        Eigen::Quaterniond quatuam4{left_fisheye_camera["attitude"]["w"].asDouble(),
                                    left_fisheye_camera["attitude"]["x"].asDouble(),
                                    left_fisheye_camera["attitude"]["y"].asDouble(),
                                    left_fisheye_camera["attitude"]["z"].asDouble()};
        

        T[0] << front_fisheye_camera["translation"]["x"].asDouble(),
                       front_fisheye_camera["translation"]["y"].asDouble(),
                       front_fisheye_camera["translation"]["z"].asDouble();
        T[1] << right_fisheye_camera["translation"]["x"].asDouble(),
                       right_fisheye_camera["translation"]["y"].asDouble(),
                       right_fisheye_camera["translation"]["z"].asDouble();
        T[2] << rear_fisheye_camera["translation"]["x"].asDouble(),
                       rear_fisheye_camera["translation"]["y"].asDouble(),
                       rear_fisheye_camera["translation"]["z"].asDouble();
        T[3] << left_fisheye_camera["translation"]["x"].asDouble(),
                       left_fisheye_camera["translation"]["y"].asDouble(),
                       left_fisheye_camera["translation"]["z"].asDouble();

        R[0] = quatuam1.toRotationMatrix();
        R[1] = quatuam2.toRotationMatrix();
        R[2] = quatuam3.toRotationMatrix();
        R[3] = quatuam4.toRotationMatrix();

        construct_homograph();
    }

    void cameraParam::construct_homograph(){

        for(int i=0; i<CAMERA_NUMS; i++){
            Eigen::Vector3d t_bc1 = T[i] + Eigen::Vector3d(-distance_vehicle_rear_center_to_car_mid, 0, 0);
            Eigen::Matrix3d R_cb = R[i].transpose();
            Eigen::Vector3d t_cb = -R_cb * t_bc1;
            Eigen::Matrix3d H_cb = R_cb - t_cb * Eigen::Vector3d(0, 0, 1.0).transpose() / (0.5 * wheel_diameter);

            H[i] = H_cb.inverse();
            H[i]/=H[i](2, 2);
        }

    }

}
