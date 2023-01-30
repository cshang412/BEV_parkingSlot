//
// Created by chenghe on 11/12/22.
//
#include "vision_parking_slot_processor.h"
#include "tool.h"
namespace psd_mapping
{
    bool PSDProcessor::process_three_visible_point_slot(ParkingSlot& slot)
    {
        int corner_num = 4;
        int angle_index = -1;
        int no_visual_index = 0;
        for(int i=0;i<corner_num;i++)
        {
            if(!slot.psd_point_body[i].visible)
            {
                angle_index = (i+2)%corner_num;
                no_visual_index = i;
                break;
            }
        }
        int short_index = (angle_index+1)%corner_num;
        int long_index = (angle_index+3)%corner_num;
        double short_dis = (slot.psd_point_body[angle_index].position - slot.psd_point_body[short_index].position).norm();
        double long_dis = (slot.psd_point_body[angle_index].position - slot.psd_point_body[long_index].position).norm();
        if(long_dis < short_dis)
        {
            int temp = short_index;
            short_index = long_index;
            long_index = temp;
            double temp_dis = long_dis;
            long_dis = short_dis;
            short_dis = temp_dis;
        }
        if(short_dis > PSDMappingParam::get_instance()->SLOT_WIDTH_HIGH_THRESHOLD ||
           short_dis < PSDMappingParam::get_instance()->SLOT_WIDTH_LOW_THRESHOLD)
        {
            return false;
        }
        if(long_dis > PSDMappingParam::get_instance()->SLOT_LENGTH_HIGH_THRESHOLD ||
           long_dis < PSDMappingParam::get_instance()->SLOT_LENGTH_LOW_THRESHOLD)
        {
            return false;
        }
        auto temp_corners_body  = slot.psd_point_body;
//        auto temp_corners_image = slot.psd_point_image;
        temp_corners_body[0] = slot.psd_point_body[short_index];
        temp_corners_body[1] = slot.psd_point_body[angle_index];
        temp_corners_body[2] = slot.psd_point_body[long_index];
        temp_corners_body[3] = slot.psd_point_body[no_visual_index];

//        temp_corners_image[0] = slot.psd_point_image[short_index];
//        temp_corners_image[1] = slot.psd_point_image[angle_index];
//        temp_corners_image[2] = slot.psd_point_image[long_index];
//        temp_corners_image[3] = slot.psd_point_image[no_visual_index];

        Eigen::Vector3d short_border1 = slot.psd_point_body[short_index].position - slot.psd_point_body[angle_index].position;
        Eigen::Vector3d long_border1 = slot.psd_point_body[long_index].position - slot.psd_point_body[angle_index].position;
        Eigen::Vector3d long_border2 = slot.psd_point_body[no_visual_index].position - slot.psd_point_body[short_index].position;
        double short_angle1 = getBorderAngle(long_border1,short_border1);
        double short_angle2 = getBorderAngle(long_border2,short_border1);
        double angle = getBorderAngle(long_border1,long_border2);


        if(std::abs(short_angle1 - M_PI/2) > PSDMappingParam::get_instance()->ANGLE_BETWEEN_PERPENDICULAR_RAD)
        {
            std::cout<<"short angle: "<<std::abs(short_angle1 - M_PI/2)/M_PI_2*180.0<<" "
                     <<std::abs(short_angle2 - M_PI/2)/M_PI_2*180.0<<std::endl;
            std::cout<<"long angle: "<<angle/M_PI_2*180.0<<std::endl;
            return false;
        }

        if(std::abs(short_angle2 - M_PI/2) > PSDMappingParam::get_instance()->ANGLE_BETWEEN_PERPENDICULAR_RAD)
        {
            std::cout<<"short angle: "<<std::abs(short_angle1 - M_PI/2)/M_PI_2*180.0<<" "
                     <<std::abs(short_angle2 - M_PI/2)/M_PI_2*180.0<<std::endl;
            std::cout<<"long angle: "<<angle/M_PI_2*180.0<<std::endl;
            return false;
        }

        double long_border_dis = long_border1.norm();
        double no_visual_long_border_dis = long_border2.norm();
        Eigen::Vector3d direction = long_border1.normalized();
        if(no_visual_long_border_dis > 0.6 * long_border_dis)
        {

            if(angle > PSDMappingParam::get_instance()->ANGLE_BETWEEN_PARALLEL_RAD)
            {
                std::cout<<"short angle: "<<std::abs(short_angle1 - M_PI/2)/M_PI_2*180.0<<" "
                         <<std::abs(short_angle2 - M_PI/2)/M_PI_2*180.0<<std::endl;
                std::cout<<"long angle: "<<angle/M_PI_2*180.0<<std::endl;
                return false;
            }

            direction = ( (long_border1 + long_border2 ) * 0.5).normalized();
        }

        temp_corners_body[3].position = direction * long_border_dis + slot.psd_point_body[short_index].position;

        slot.psd_point_body = temp_corners_body;
//        slot.psd_point_image = temp_corners_image;

        return true;
    }
    bool PSDProcessor::process_two_visible_point_slot(ParkingSlot& slot)
    {
        double distance = 0;
        int visual_index = -1;
        int corner_num = 4;
        for(int i=0;i<corner_num;i++)
        {
            int next_idx = (i+1)%corner_num;
            if(slot.psd_point_body[i].visible && slot.psd_point_body[next_idx].visible)
            {
                visual_index = i;
                distance = (slot.psd_point_body[next_idx].position-slot.psd_point_body[i].position).norm();
            }
        }
        if(visual_index < 0)
        {
            return false;
        }
        int index1 = visual_index;
        int index2 = (visual_index+1)%corner_num;
        int index3 = (visual_index+2)%corner_num;
        int index4 = (visual_index+3)%corner_num;
        Eigen::Vector3d border1 = slot.psd_point_body[index4].position-slot.psd_point_body[index1].position;
        Eigen::Vector3d border2 = slot.psd_point_body[index3].position-slot.psd_point_body[index2].position;
        Eigen::Vector3d direction = ((border1 + border2) * 0.5).normalized();
        Eigen::Vector3d short_border = slot.psd_point_body[index2].position-slot.psd_point_body[index1].position;
        double short_angle1 = getBorderAngle(border1,short_border);
        double short_angle2 = getBorderAngle(border2,short_border);
        double angle = getBorderAngle(border1,border2);
        if(angle > PSDMappingParam::get_instance()->ANGLE_BETWEEN_PARALLEL_RAD)
        {
            std::cout<<"short angle: "<<std::abs(short_angle1 - M_PI/2)/M_PI_2*180.0<<" "
                     <<std::abs(short_angle2 - M_PI/2)/M_PI_2*180.0<<std::endl;
            std::cout<<"long angle: "<<angle/M_PI_2*180.0<<std::endl;
            return false;
        }

        if(std::abs(short_angle1 - M_PI/2) > PSDMappingParam::get_instance()->ANGLE_BETWEEN_PERPENDICULAR_RAD)
        {
            std::cout<<"short angle: "<<std::abs(short_angle1 - M_PI/2)/M_PI_2*180.0<<" "
                     <<std::abs(short_angle2 - M_PI/2)/M_PI_2*180.0<<std::endl;
            std::cout<<"long angle: "<<angle/M_PI_2*180.0<<std::endl;
            return false;
        }

        if(std::abs(short_angle2 - M_PI/2) > PSDMappingParam::get_instance()->ANGLE_BETWEEN_PERPENDICULAR_RAD)
        {
            std::cout<<"short angle: "<<std::abs(short_angle1 - M_PI/2)/M_PI_2*180.0<<" "
                     <<std::abs(short_angle2 - M_PI/2)/M_PI_2*180.0<<std::endl;
            std::cout<<"long angle: "<<angle/M_PI_2*180.0<<std::endl;
            return false;
        }

        int visual_boundary = 0;
        if(PSDMappingParam::get_instance()->SLOT_WIDTH_LOW_THRESHOLD < distance &&
           distance < PSDMappingParam::get_instance()->SLOT_WIDTH_HIGH_THRESHOLD)
        {
            visual_boundary = 1;//短边
        }
        else if(PSDMappingParam::get_instance()->SLOT_LENGTH_LOW_THRESHOLD < distance &&
                distance < PSDMappingParam::get_instance()->SLOT_LENGTH_HIGH_THRESHOLD)
        {
            visual_boundary = 2;//长边
        }
        if(visual_boundary == 0)
        {
            return false;
        }
        std::unordered_set<int> us;
        for(int i=0;i<corner_num;i++)
        {
            int next_idx = (i+1)%corner_num;

            if(slot.psd_point_body[i].visible && !slot.psd_point_body[next_idx].visible)
            {
                if(us.count(next_idx))
                {
                    continue;
                }
                us.insert(next_idx);
                gtsam::Vector3 dir = (slot.psd_point_body[next_idx].position - slot.psd_point_body[i].position).normalized();
                //当前点可见，下个点不可见。
                if(visual_boundary == 1)
                {
                    //长边不可见
                    slot.psd_point_body[next_idx].position = slot.psd_point_body[i].position + dir *
                                                                                               PSDMappingParam::get_instance()->SLOT_LENGTH;
                }

                if(visual_boundary == 2)
                {
                    slot.psd_point_body[next_idx].position = slot.psd_point_body[i].position + dir *
                                                                                               PSDMappingParam::get_instance()->SLOT_WIDTH;
                }
            }

            if(!slot.psd_point_body[i].visible && slot.psd_point_body[next_idx].visible)
            {
                if(us.count(i))
                {
                    continue;
                }
                us.insert(i);
                //当前点不可见，下个点可见。
                gtsam::Vector3 dir = (slot.psd_point_body[i].position - slot.psd_point_body[next_idx].position).normalized();
                if(visual_boundary == 1)
                {
                    //长边不可见
                    slot.psd_point_body[i].position = slot.psd_point_body[next_idx].position + dir *
                                                                                                      PSDMappingParam::get_instance()->SLOT_LENGTH;
                }

                if(visual_boundary == 2)
                {
                    slot.psd_point_body[i].position = slot.psd_point_body[next_idx].position + dir *
                                                                                                      PSDMappingParam::get_instance()->SLOT_WIDTH;
                }
            }
        }

        return true;
    }

    bool PSDProcessor::process(ParkingSlot& slot)
    {
        if(slot.psd_point_image.size() != 4)
        {
            return false;
        }

        int num_visible = 0;
        for(auto& p: slot.psd_point_image)
        {
            if(p.visible)
            {
                num_visible++;
            }
        }

        if(num_visible < 2)
        {
            return false;
        }
        slot.center_body = slot.center_image;
        for(int i = 0; i < slot.psd_point_image.size(); ++i)
        {
            ParkingSlotPoint3 pbody;
            pbody = slot.psd_point_image[i];
            slot.psd_point_body.push_back(pbody);
        }
        slot.area = IOU::areaEx(slot.get_body_vertexes());

        if(slot.area/ 10.0 < 0.3)
        {
            return false;
        }

        if(num_visible == 2)
        {
            if(!process_two_visible_point_slot(slot))
            {
                return false;
            }
        }

        if(num_visible == 3)
        {
            if(!process_three_visible_point_slot(slot))
            {
                return false;
            }
        }

        return true;
    }

    bool PSDProcessor::processv2(ParkingSlot& slot)
    {
        if(slot.psd_point_body.size() != 4)
        {
            return false;
        }

        int num_visible = 0;
        for(auto& p: slot.psd_point_body)
        {
            if(p.visible)
            {
                num_visible++;
            }
        }

        if(num_visible < 2)
        {
            return false;
        }
        slot.area = IOU::areaEx(slot.get_body_vertexes());

        if(slot.area/ 10.0 < 0.3)
        {
            return false;
        }

        if(num_visible == 2)
        {
            if(!process_two_visible_point_slot(slot))
            {
                return false;
            }
        }

        if(num_visible == 3)
        {
            if(!process_three_visible_point_slot(slot))
            {
                return false;
            }
        }

        return true;
    }

    std::vector<PSDAssociation> PSDProcessor::
    cluster_parking_slots(const std::vector<ParkingSlotFrame>& psd_frames,
                         std::double_t min_iou)
    {
        std::vector<PSDAssociation> clusters;
        std::vector<ParkingSlot> slots;
        for(const auto& psd_frame: psd_frames)
        {
            for (const auto &psd: psd_frame.park_slots) {
                if(!psd.psd_point_world.empty())
                {
                    slots.push_back(psd);
                }
            }
        }
        std::unordered_set<int> us;
        int track_id = 0;
        for(int i = 0; i < slots.size(); ++i)
        {
            auto vertexes_i = slots[i].get_world_vertexes();
            PSDAssociation cluster;
            cluster.id = track_id;
            cluster.slots.push_back(slots[i]);
            for(int j = i+1; j < slots.size(); ++j)
            {
                if(us.count(j))
                {
                    continue;
                }
                auto vertexes_j = slots[j].get_world_vertexes();

                auto iou = IOU::iouEx(vertexes_i, vertexes_j);
                if(iou > min_iou)
                {
                    us.insert(j);
                    cluster.slots.push_back(slots[j]);
                }
            }
            if(cluster.slots.size() <= 2)
            {
                continue;
            }
            track_id++;
            clusters.push_back(cluster);
        }
        for(auto& cluster: clusters)
        {
            int max_area_id = 0;
            double max_area = 0.0;
            for(int i = 0; i < cluster.slots.size(); ++i)
            {
                if(cluster.slots[i].area > max_area)
                {
                    max_area = cluster.slots[i].area;
                    max_area_id = i;
                }
            }
            std::swap(cluster.slots[0], cluster.slots[max_area_id]);
        }

        for(auto& cluster: clusters)
        {
            for(int i = 1; i < cluster.slots.size(); ++i)
            {
                std::double_t min_distance = std::numeric_limits<double>::max();
                std::vector<int>  min_indices  = {0, 1, 2, 3};
                std::vector<std::vector<int>> indices_vector
                                                        {{0, 1, 2, 3},
                                                         {0, 1, 3, 2},
                                                         {0, 2, 1, 3},
                                                         {0, 2, 3, 1},
                                                         {0, 3, 1, 2},
                                                         {0, 3, 2, 1},

                                                         {1, 0, 2, 3},
                                                         {1, 0, 3, 2},
                                                         {1, 2, 0, 3},
                                                         {1, 2, 3, 0},
                                                         {1, 3, 0, 2},
                                                         {1, 3, 2, 0},

                                                         {2, 0, 1, 3},
                                                         {2, 0, 3, 1},
                                                         {2, 1, 0, 3},
                                                         {2, 1, 3, 0},
                                                         {2, 3, 0, 1},
                                                         {2, 3, 1, 0},

                                                         {3, 0, 1, 2},
                                                         {3, 0, 2, 1},
                                                         {3, 1, 0, 2},
                                                         {3, 1, 2, 0},
                                                         {3, 2, 0, 1},
                                                         {3, 2, 1, 0}};
                for(const auto& indices: indices_vector)
                {
                    std::double_t distance = 0;
                    for(int k = 0; k < 4; ++k)
                    {
                        distance += (cluster.slots[0].psd_point_world[k].position -
                                     cluster.slots[i].psd_point_world[indices[k]].position).norm();
                    }
                    if(distance < min_distance)
                    {
                        min_distance = distance;
                        min_indices  = indices;
                    }
                }
                std::cout<<"cluster id: "<<cluster.id<<", j = "<<i
                         <<", min_distance: "<<min_distance/4.0<<std::endl;
                auto temp_corners_world = cluster.slots[i].psd_point_world;
                auto temp_corners_body  = cluster.slots[i].psd_point_body;
                auto temp_corners_image = cluster.slots[i].psd_point_image;
                for(int k = 0; k < 4; ++k)
                {
                    temp_corners_world[k] = cluster.slots[i].psd_point_world[min_indices[k]];
                    temp_corners_body[k]  = cluster.slots[i].psd_point_body[min_indices[k]];
                    temp_corners_image[k] = cluster.slots[i].psd_point_image[min_indices[k]];
                }

                cluster.slots[i].psd_point_world = temp_corners_world;
                cluster.slots[i].psd_point_body  = temp_corners_body;
                cluster.slots[i].psd_point_image = temp_corners_image;
            }
        }

        return clusters;
    }

    void PSDProcessor::draw_cluster(const std::vector<PSDAssociation>& clusters,
                                    const std::string& fusion_path)
    {
        std::random_device rd;
        std::default_random_engine eng(rd());
        std::uniform_real_distribution<float> distr(0.0, 254.0);

        pcl::PointCloud<pcl::PointXYZRGB> pts;
        pcl::PointCloud<pcl::PointXYZRGB> pts_fusion;
        for(const auto& cluster: clusters)
        {
            std::uint8_t r = static_cast<std::uint8_t>( std::round(distr(eng)));
            std::uint8_t g = static_cast<std::uint8_t>( std::round(distr(eng)));
            std::uint8_t b = static_cast<std::uint8_t>( std::round(distr(eng)));

            Tool::draw_slot(cluster.fusion_points_world, 0.1, pts_fusion, r, g, b);
        }
        pcl::io::savePLYFile(fusion_path, pts_fusion);
    }

    void PSDProcessor::merge_cluster(std::vector<PSDAssociation>& clusters)
    {
        for(auto& cluster: clusters)
        {
            cluster.fusion_points_world = cluster.slots[0].psd_point_world;
            std::vector<std::double_t> cnt{cluster.fusion_points_world[0].confidence,
                                           cluster.fusion_points_world[1].confidence,
                                           cluster.fusion_points_world[2].confidence,
                                           cluster.fusion_points_world[3].confidence};
            for(int k = 0; k < 4; ++k)
            {
                cluster.fusion_points_world[k].position *=
                        cluster.fusion_points_world[k].confidence;
            }
            for(int i = 1; i < cluster.slots.size(); ++i)
            {
                for(int k = 0; k < 4; ++k)
                {
//                    if(cluster.slots[i].psd_point_world[k].visible)
//                    {
//                        cluster.fusion_points_world[k].position +=
//                                cluster.slots[i].psd_point_world[k].position *
//                                cluster.slots[i].psd_point_world[k].confidence;
//                        cnt[k] += cluster.slots[i].psd_point_world[k].confidence;
//                    }

                    cluster.fusion_points_world[k].position +=
                            cluster.slots[i].psd_point_world[k].position *
                            cluster.slots[i].psd_point_world[k].confidence;
                    cnt[k] += cluster.slots[i].psd_point_world[k].confidence;
                }
            }

            for(int k = 0; k < 4; ++k)
            {
                std::cout<<cluster.fusion_points_world[k].position.transpose()<<std::endl;
                cluster.fusion_points_world[k].position /= cnt[k];
                std::cout<<cluster.fusion_points_world[k].position.transpose()<<std::endl;
            }

//            std::cout<<"cluster: "<<cluster.id<<" ["
//                     <<cnt[0]<<" "<<cnt[1]<<" "<<cnt[2]<<" "<<cnt[3]<<"]"<<std::endl;
        }
    }
}