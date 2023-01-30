//
// Created by chenghe on 11/12/22.
//

#ifndef PSD_MAPPING_PSD_PARAMS_H
#define PSD_MAPPING_PSD_PARAMS_H
#include "psd_utils.h"
namespace psd_mapping
{
    class PSDMappingParam
    {
    public:
        static PSDMappingParam* get_instance()
        {
            static PSDMappingParam param;
            return  &param;
        }

        std::double_t DISTANCE_REAR_AXLE_MID_TO_CAR_CENTER = 1.5;

        std::double_t IPM_IMAGE_SIZE                       = 1200.0;
        std::double_t IPM_PIXEL_SIZE                       = 0.02;

        std::double_t SLOT_LENGTH = 5.2;
        std::double_t SLOT_WIDTH  = 2.5;

        std::double_t SLOT_LENGTH_LOW_THRESHOLD   = 5.0;
        std::double_t SLOT_LENGTH_HIGH_THRESHOLD  = 6.0;
        std::double_t SLOT_WIDTH_LOW_THRESHOLD    = 1.8;
        std::double_t SLOT_WIDTH_HIGH_THRESHOLD   = 3.0;

        std::double_t ANGLE_BETWEEN_PARALLEL_RAD      = 100.0 * M_PI / 180;
        std::double_t ANGLE_BETWEEN_PERPENDICULAR_RAD = 100.0 * M_PI / 180;
    };

    class cameraParam
    {
    public:
        void load_cameraparam(const std::string& path);

        int CAMERA_NUMS = 4;
        double distance_vehicle_rear_center_to_car_mid = 1.5;
        double wheel_diameter = 0.786;

        std::vector<Eigen::Vector3d> T = std::vector<Eigen::Vector3d>(4);   //body to camera
        std::vector<Eigen::Matrix3d> R = std::vector<Eigen::Matrix3d>(4);   //body to camera
        std::vector<Eigen::Matrix3d> H = std::vector<Eigen::Matrix3d>(4);   //camera to bev Homography
        
    private:
        void construct_homograph();
    };
}

#endif //PSD_MAPPING_PSD_PARAMS_H
