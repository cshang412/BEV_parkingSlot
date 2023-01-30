//
// Created by chenghe on 11/11/22.
//
#include "psd_frame.h"
#include "vision_parking_slot_processor.h"
namespace psd_mapping
{
    gtsam::Key LocFrame::get_key() const
    {
        gtsam::symbol_shorthand::X(static_cast<uint64_t>(timestamp*1e6));
    }
    void LocFrame::set_pose(const Eigen::Quaterniond& q,
                            const Eigen::Vector3d&    t)
    {
        pose = gtsam::Pose3(gtsam::Rot3(q), t);
    }
    FrameType LocFrame::get_type() const
    {
        return FrameType::LOC;
    }

    bool LocFrame::load_from_json(const std::string& file_name)
    {
        auto value = Util::load_json(file_name);
        timestamp = value["meta"]["timestamp_us"].asInt64()*1e-6;
        Eigen::Quaterniond q(value["orientation"]["quaternion_local"]["w"].asDouble(),
                             value["orientation"]["quaternion_local"]["x"].asDouble(),
                             value["orientation"]["quaternion_local"]["y"].asDouble(),
                             value["orientation"]["quaternion_local"]["z"].asDouble());
        Eigen::Vector3d t(value["position"]["position_local"]["x"].asDouble(),
                          value["position"]["position_local"]["y"].asDouble(),
                          value["position"]["position_local"]["z"].asDouble());
        set_pose(q, t);

        return true;
    }

    ParkingSlotPoint3::ParkingSlotPoint3():
            ParkingSlotPointBase()
    {

    }

    ParkingSlotPoint3& ParkingSlotPoint3::operator = (const ParkingSlotPoint2& other)
    {
        visible    = other.visible;
        confidence = other.confidence;
        double x = PSDMappingParam::get_instance()->IPM_PIXEL_SIZE*(-other.position.y() + PSDMappingParam::get_instance()->IPM_IMAGE_SIZE/2.0) +
                   PSDMappingParam::get_instance()->DISTANCE_REAR_AXLE_MID_TO_CAR_CENTER;
        double y = PSDMappingParam::get_instance()->IPM_PIXEL_SIZE*(-other.position.x() + PSDMappingParam::get_instance()->IPM_IMAGE_SIZE/2.0);

        position = gtsam::Point3(x, y, 0);

        return *this;
    }

    void ParkingSlotPoint3::transform(const gtsam::Pose3& pose)
    {
        position = pose.transformFrom(position);
    }

    ParkingSlotPoint2::ParkingSlotPoint2():
            ParkingSlotPointBase()
    {

    }

    IOU::Vertexes ParkingSlot::get_image_vertexes() const
    {
        return get_vertexes(psd_point_image);
    }

    IOU::Vertexes ParkingSlot::get_body_vertexes() const
    {
        return get_vertexes(psd_point_body);
    }

    IOU::Vertexes ParkingSlot::get_world_vertexes() const
    {
        return get_vertexes(psd_point_world);
    }

    void ParkingSlot::transform_to_world(const gtsam::Pose3& pose)
    {
        for(const auto& pbody: psd_point_body)
        {
            ParkingSlotPoint3 pw = pbody;
            pw.transform(pose);
            psd_point_world.push_back(pw);
        }
        center_world = center_body;
        center_world.transform(pose);
    }

    ParkingSlotFrame::ParkingSlotFrame(FrameType type):
        type_(type)
    {

    }

    void ParkingSlotFrame::transform_to_world(const gtsam::Pose3& pose)
    {
        for(auto& slot: park_slots)
        {
            slot.transform_to_world(pose);
        }
    }

    bool ParkingSlotFrame::load_from_json(const std::string& file_name)
    {
        if(type_ == FrameType::PSD_FUSION)
        {
            return load_fusion_frame(file_name);
        }
        else if(type_ == FrameType::PSD_VISION)
        {
            return load_vision_frame(file_name);
        }
        else
        {
            std::cerr<<"invalid psd frame type"<<std::endl;
            throw std::exception{};
        }

        return false;
    }

    bool ParkingSlotFrame::load_vision_frame(const std::string& file_name)
    {
        auto value = Util::load_json(file_name);
        timestamp = value["meta"]["sensor_timestamp_us"].asInt64()*1e-6;
        for(int i = 0; i < value["parking_slots"].size(); ++i)
        {

            auto v = value["parking_slots"][i];
            if(v["corner_points"].size()  != 4)
            {
                continue;
            }
            ParkingSlot slot;
            slot.timestamp = timestamp;
            slot.center_image.visible    = true;
            slot.center_image.confidence = v["center_point"]["confidence"].asDouble();
            slot.center_image.position = gtsam::Point2(v["center_point"]["position"]["x"].asDouble(),
                                                       v["center_point"]["position"]["y"].asDouble());
            for(int j = 0; j < v["corner_points"].size(); ++j)
            {
                slot.psd_point_image.emplace_back();
                slot.psd_point_image.back().visible    = v["corner_points"][j]["type"]["value"].asInt() == 1;
                slot.psd_point_image.back().confidence = v["corner_points"][j]["confidence"].asDouble();
                slot.psd_point_image.back().position   = gtsam::Point2(v["corner_points"][j]["position"]["x"].asDouble(),
                                                                       v["corner_points"][j]["position"]["y"].asDouble());
            }

            if(PSDProcessor::process(slot))
            {
                park_slots.push_back(slot);
            }
        }

        return true;
    }

    bool ParkingSlotFrame::load_fusion_frame(const std::string& file_name)
    {
//        auto value = Util::load_json(file_name);
//        timestamp = value["meta"]["sensor_timestamp_us"].asInt64()*1e-6;
//        for(int i = 0; i < value["parking_slot"]["parking_slot_data"].size(); ++i)
//        {
//            auto v = value["parking_slot"]["parking_slot_data"][i];
//            ParkingSlot3D parking_slot;
//            parking_slot.track_id = v["track_id"].asInt();
//            for(int j = 0; j < v["local_points_fusion"].size(); ++j)
//            {
//                auto pts = v["local_points_fusion"][j];
//                psd.pts.push_back(Eigen::Vector3d{pts["x"].asDouble(), pts["y"].asDouble(), pts["z"].asDouble()});
//            }
//
//            psd_frame.psds.push_back(psd);
//        }
//
//        return psd_frame;

        return true;
    }

    FrameType ParkingSlotFrame::get_type() const
    {
        return type_;
    }
}