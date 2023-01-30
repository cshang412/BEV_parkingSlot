//
// Created by chenghe on 11/18/22.
//

#ifndef PSD_MAPPING_PSD_OPTIMIZER_H
#define PSD_MAPPING_PSD_OPTIMIZER_H

#include "vision_parking_slot_processor.h"
#include "ceres/ceres.h"
#include "hardware_config.h"

namespace psd_mapping
{
    class PSDOptimizer
    {
    public:
        static bool process(std::vector<PSDAssociation> &clusters, cameraParam camera_param, HardwareConfig hardware_config);

        static double get_angle(const gtsam::Vector3& a,
                                const gtsam::Vector3& b)
        {
            double alpha = acos(a.dot(b)/(a.norm()*b.norm()));
            if(alpha > M_PI/2)
            {
                return M_PI - alpha;
            }
            else
            {
                return alpha;
            }
        }
    };
}

#endif //PSD_MAPPING_PSD_OPTIMIZER_H
