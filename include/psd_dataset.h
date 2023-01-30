//
// Created by chenghe on 11/11/22.
//

#ifndef ODOMETER_PSD_DATASET_H
#define ODOMETER_PSD_DATASET_H

#include "psd_frame.h"

namespace psd_mapping
{
    class PSDDataset
    {
    public:

        bool load(const std::string& path);

        bool load_vision_psd(const std::string& path);

        bool load_loc(const std::string& path);

        bool load_loc_v2(const std::string& path);

        bool transform_vision_frame_to_world();

        void draw_vision_slot(const std::string& ply_file_name) const;

        void draw_taj(const std::string& ply_file_name) const;

        bool get_pose(std::double_t ts, gtsam::Pose3& pose) const;

        gtsam::Point2 body2bev(gtsam::Point3& point);

        std::vector<LocFrame>         loc_frame_vector;
        std::vector<ParkingSlotFrame> psd_frame_vision_vector;
        std::vector<ParkingSlotFrame> psd_frame_fusion_vector;
    };
}

#endif //ODOMETER_PSD_DATASET_H
