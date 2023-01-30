//
// Created by chenghe on 11/12/22.
//

#ifndef ODOMETER_VISION_PARKING_SLOT_PROCESSOR_H
#define ODOMETER_VISION_PARKING_SLOT_PROCESSOR_H

#include "psd_frame.h"



namespace psd_mapping
{
    class PSDAssociation
    {
    public:
        std::uint32_t id;
        std::vector<ParkingSlot> slots;
        std::vector<ParkingSlotPoint3> fusion_points_world;

        std::vector<gtsam::Point2> transform_to_body(const gtsam::Pose3& pose) const
        {
            std::vector<gtsam::Point2> pt2s;
            for(const auto& p: fusion_points_world)
            {
                gtsam::Point3 pb = pose.transformTo(p.position);
                double x = (pb.x() - 1.5)/0.02 - 600;
                double y = (pb.y())/0.02 - 600;

                pt2s.push_back(gtsam::Point2(-y, -x));
            }

            return pt2s;
        }

        std::vector<gtsam::Point3> transform_to_vbody(const gtsam::Pose3& pose) const
        {
            std::vector<gtsam::Point3> pt2s;
            for(const auto& p: fusion_points_world)
            {
                gtsam::Point3 pb = pose.transformTo(p.position);
                pt2s.push_back(pb);
            }

            return pt2s;
        }

        static void gtsam_point3_to_array(const gtsam::Point3& a,
                                          std::array<double, 3>& b)
        {
            b[0] = a.x(); b[1] = a.y(); b[2] = a.z();
        }

        static void array_to_gtsam_point3(gtsam::Point3& a,
                                          const std::array<double, 3>& b)
        {
            a = gtsam::Point3(b[0], b[1], b[2]);
        }

        void init_data_array()
        {
            gtsam_point3_to_array(fusion_points_world[0].position, p1_data);
            gtsam_point3_to_array(fusion_points_world[1].position -
                                  fusion_points_world[0].position, d12_data);
            gtsam_point3_to_array(fusion_points_world[2].position -
                                  fusion_points_world[1].position, d23_data);
        }

        void update()
        {
            fusion_points_world[0].position = gtsam::Point3(p1_data[0], p1_data[1], p1_data[2]);
            fusion_points_world[1].position = fusion_points_world[0].position +
                    gtsam::Point3(d12_data[0], d12_data[1], d12_data[2]);
            fusion_points_world[2].position = fusion_points_world[1].position +
                    gtsam::Point3(d23_data[0], d23_data[1], d23_data[2]);
            fusion_points_world[3].position = fusion_points_world[0].position +
                    gtsam::Point3(d23_data[0], d23_data[1], d23_data[2]);
        }

        std::array<double, 3> p1_data;
        std::array<double, 3> d12_data;
        std::array<double, 3> d23_data;

        gtsam::Vector3 get_dir12() const
        {
            return fusion_points_world[1].position -
                   fusion_points_world[0].position;
        }

        gtsam::Vector3 get_dir23() const
        {
            return fusion_points_world[2].position -
                   fusion_points_world[1].position;
        }
    };

    class PSDProcessor
    {
    public:

        static bool process_two_visible_point_slot(ParkingSlot& slot);

        static bool process_three_visible_point_slot(ParkingSlot& slot);

        static bool process(ParkingSlot& slot);

        static bool processv2(ParkingSlot& slot);

        static std::vector<PSDAssociation> cluster_parking_slots(const std::vector<ParkingSlotFrame>& psd_frames,
                                                                 std::double_t min_iou = 0.4);

        static void draw_cluster(const std::vector<PSDAssociation>& clusters,
                                 const std::string& fusion_path);

        static void merge_cluster(std::vector<PSDAssociation>& clusters);

        static double getBorderAngle(const Eigen::Vector3d& border1, const Eigen::Vector3d& border2)
        {
            double length1 = border1.norm();
            double length2 = border2.norm();
            if(length1 < 1e-4 || length2 < 1e-4)
            {
                return 0;
            }
            double alpha = acos(border1.dot(border2)/(length1*length2));
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

#endif //ODOMETER_VISION_PARKING_SLOT_PROCESSOR_H
