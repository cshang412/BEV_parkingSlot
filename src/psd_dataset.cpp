//
// Created by chenghe on 11/11/22.
//
#include "psd_dataset.h"
#include "tool.h"
#include "psd_params.h"
#include "vision_parking_slot_processor.h"
namespace psd_mapping
{

    gtsam::Point2 PSDDataset::body2bev(gtsam::Point3& point)
    {
        std::double_t pixel_size = PSDMappingParam::get_instance()->IPM_PIXEL_SIZE;
        std::double_t ipm_size = PSDMappingParam::get_instance()->IPM_IMAGE_SIZE;
        std::double_t car2center = PSDMappingParam::get_instance()->DISTANCE_REAR_AXLE_MID_TO_CAR_CENTER;

        return gtsam::Point2(((-point.x()+car2center)/pixel_size + ipm_size/2.0),
                            (-point.y()/pixel_size + ipm_size/2.0));
    }


    bool PSDDataset::load(const std::string& path)
    {
        std::vector<std::string> filenames;
        Util::get_filenames(path, filenames);
        for(const auto& f: filenames)
        {
//            std::cout<<f<<std::endl;
            auto result = Util::split_string(f, "/");
            if(result.empty())
            {
                continue;
            }
            auto file_name_split = Util::split_string(result.back(), ".");
            if(file_name_split.size() != 2)
            {
                continue;
            }
            auto file_name = file_name_split.front();
            auto other_split = Util::split_string(file_name, "_");
            if(other_split.size() < 2)
            {
                continue;
            }
            auto timestamp = std::stoll(other_split.front());
            auto type = other_split[1];
//            std::cout<<"ts: "<<timestamp<<", type: "<<type<<std::endl;
            if(type == "fuison")
            {
                ParkingSlotFrame psd_fusion_frame(FrameType::PSD_FUSION);
                if(psd_fusion_frame.load_from_json(f))
                {
                    psd_frame_fusion_vector.push_back(psd_fusion_frame);
                }
            }

            if(type == "vision")
            {
                ParkingSlotFrame psd_fusion_frame(FrameType::PSD_VISION);
                if(psd_fusion_frame.load_from_json(f))
                {
                    psd_frame_vision_vector.push_back(psd_fusion_frame);
                }
            }

//            if(type == "loc")
//            {
//                LocFrame loc_frame;
//                if(loc_frame.load_from_json(f))
//                {
//                    loc_frame_vector.push_back(loc_frame);
//                }
//            }
        }
//        std::sort(loc_frame_vector.begin(),        loc_frame_vector.end(),        FrameCMP());
        std::sort(psd_frame_vision_vector.begin(), psd_frame_vision_vector.end(), FrameCMP());
        std::sort(psd_frame_fusion_vector.begin(), psd_frame_fusion_vector.end(), FrameCMP());

        return true;
    }

    bool PSDDataset::load_vision_psd(const std::string& path)
    {
        auto root = Util::load_json(path);
        std::cout<<root["frame"].size()<<std::endl;
        for (const auto& psd_json: root["frame"])
        {
            std::cout<<psd_json<<std::endl;
            ParkingSlotFrame psd_frame(FrameType::PSD_VISION);
            psd_frame.timestamp = psd_json["timestamp_us"].asUInt64() * 1e-6;

            for(const auto& slot_json: psd_json["slots"][0])
            {
                ParkingSlot slot;
                slot.timestamp = psd_frame.timestamp;

                //std::vector<string> keys{"front_left", "rear_left", "rear_right", "front_right"};
                for(int i = 0; i < 4; ++i)
                {
                    //auto key = keys[i];
                    ParkingSlotPoint3 front_left_pt;
                    front_left_pt.position = gtsam::Point3(slot_json["points"][i]["x"].asDouble(),
                                                           slot_json["points"][i]["y"].asDouble(),
                                                           -0.786*0.5);
                                                           //slot_json["points"][i]["z"].asDouble()
                    int flag = slot_json["points"][i]["visibility"].asInt();
                    if(flag == 0)
                        front_left_pt.visible  = false;
                    else
                        front_left_pt.visible  = true;

                    front_left_pt.confidence = slot_json["points"][i]["scores"].asDouble();

                    slot.psd_point_body.push_back(front_left_pt);
                }

                if(PSDProcessor::processv2(slot))
                {
                    for(int i=0; i<4; i++)
                    {
                        ParkingSlotPoint2 pt2d;
                        pt2d.position = gtsam::Point2(-(slot.psd_point_body[i].position.y())/0.02 +600,
                                                    -(slot.psd_point_body[i].position.x()-1.5)/0.02 +600);
                        pt2d.confidence = slot.psd_point_body[i].confidence;
                        pt2d.visible = slot.psd_point_body[i].visible;
                        slot.psd_point_image.push_back(pt2d);
                    }
                    psd_frame.park_slots.push_back(slot);
                }
            }
            psd_frame_vision_vector.push_back(psd_frame);
        }
        std::sort(psd_frame_vision_vector.begin(), psd_frame_vision_vector.end(), FrameCMP());

        return true;
    }

    bool PSDDataset::load_loc(const std::string& path)
    {
        std::ifstream ifs(path);

        if(!ifs.is_open())
        {
            return false;
        }

        string s;
        while(getline(ifs,s))
        {
            auto result = Util::split_string(s, " ");
            if(result.empty())
            {
                continue;
            }

            LocFrame loc_frame;
            loc_frame.timestamp = std::stoll(result[0]) * 1e-6;
            Eigen::Quaterniond q(std::stod(result[4]),
                                 std::stod(result[5]),
                                 std::stod(result[6]),
                                 std::stod(result[7]));
            Eigen::Vector3d t(std::stod(result[1]), std::stod(result[2]), std::stod(result[3]));
            loc_frame.set_pose(q, t);
            loc_frame_vector.push_back(loc_frame);
        }

        std::sort(loc_frame_vector.begin(),        loc_frame_vector.end(),        FrameCMP());

        ifs.close();

        return true;
    }

    bool PSDDataset::load_loc_v2(const std::string& path)
    {
        auto root = Util::load_json(path);

        for(const auto& value: root)
        {
            std::cout<<std::stoll(value["timestamp"].asString())<<std::endl;
            std::cout<<std::endl;
            LocFrame loc_frame;
            loc_frame.timestamp = std::stoll(value["timestamp"].asString()) * 1e-6;
            Eigen::Quaterniond q(value["rotation"]["w"].asDouble(),
                                 value["rotation"]["x"].asDouble(),
                                 value["rotation"]["y"].asDouble(),
                                 value["rotation"]["z"].asDouble());
            Eigen::Vector3d t(value["translation"]["x"].asDouble(),
                              value["translation"]["y"].asDouble(),
                              value["translation"]["z"].asDouble());
            loc_frame.set_pose(q, t);
            loc_frame_vector.push_back(loc_frame);
        }

        return true;
    }

    bool PSDDataset::get_pose(std::double_t timestamp, gtsam::Pose3& pose) const
    {
        auto iter      = loc_frame_vector.begin();
        auto next_iter = std::next(iter);
        bool found = false;
        for(; iter != loc_frame_vector.end() && next_iter != loc_frame_vector.end(); ++iter, ++next_iter)
        {
            if(timestamp >= iter->timestamp && timestamp <= next_iter->timestamp)
            {
                pose =  Util::interpolate_pose(iter->pose, iter->timestamp, next_iter->pose, next_iter->timestamp, timestamp);
                found = true;
                break;
            }
        }

        return found;
    }

    bool PSDDataset::transform_vision_frame_to_world()
    {
        for(auto& psd_vision: psd_frame_vision_vector)
        {
            auto timestamp = psd_vision.timestamp;
            auto iter      = loc_frame_vector.begin();
            auto next_iter = std::next(iter);
            bool found = false;
            gtsam::Pose3 pose;
            for(; iter != loc_frame_vector.end() && next_iter != loc_frame_vector.end(); ++iter, ++next_iter)
            {
                if(timestamp >= iter->timestamp && timestamp <= next_iter->timestamp)
                {
                    pose =  Util::interpolate_pose(iter->pose, iter->timestamp, next_iter->pose, next_iter->timestamp, timestamp);
                    found = true;
                    break;
                }
            }

            if(found)
            {
                psd_vision.transform_to_world(pose);
                for(auto& per_slot: psd_vision.park_slots){
                    per_slot.pose = pose;
                }
            }
        }

        return true;
    }

    void PSDDataset::draw_vision_slot(const std::string& ply_file_name) const
    {
        pcl::PointCloud<pcl::PointXYZRGB> pts;
        for(const auto& psd_vision: psd_frame_vision_vector)
        {
            for(const auto& slot: psd_vision.park_slots)
            {
                if(slot.psd_point_world.empty())
                {
                    continue;
                }
                Tool::draw_slot(slot.psd_point_world, 0.1, pts);
            }
        }
        pcl::io::savePLYFile(ply_file_name, pts);
    }

    void PSDDataset::draw_taj(const std::string& ply_file_name) const
    {
        pcl::PointCloud<pcl::PointXYZ> pts;
        for(const auto& loc_frame: loc_frame_vector)
        {
            pts.emplace_back(loc_frame.pose.translation().x(),
                             loc_frame.pose.translation().y(),
                             loc_frame.pose.translation().z());
        }
        pcl::io::savePLYFile(ply_file_name, pts);
    }
}