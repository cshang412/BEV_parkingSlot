// Created by chenghe on 11/12/22.
//
#include "psd_params.h"
#include "hardware_config.h"
#include "psd_dataset.h"
#include "vision_parking_slot_processor.h"
#include "psd_optimizer.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <opencv2/opencv.hpp>

using namespace psd_mapping;

#include "json/json.h"


struct PairCMP
{
    bool operator ()(const std::pair<std::int64_t, std::string>& a,
                     const std::pair<std::int64_t, std::string>& b)
    {
        return a.first < b.first;
    }
};



int main()
{
    const std::double_t ipm_size = PSDMappingParam::get_instance()->IPM_IMAGE_SIZE;

    const std::string data_root_path = "/home/chen/mydata/1116/";
    const std::string bag_path = "hibag_HP-30-V71-AC-017_default_002_20221116222600_20221116222708";

    const std::string project_path = data_root_path + "/" + bag_path;
    const std::string loc_path = project_path + "/trajectory.json";
    const std::string cameraParam_path = project_path + "/hardware_config.json";
    const std::string vision_psd_path = project_path + "/vision_psd.ply";
    const std::string optimized_psd_path = project_path + "/optimized_psd.ply";
    const std::string fusion_psd_path = project_path + "/fusion_psd.ply";

    HardwareConfig hardware_config;
    hardware_config.load_from_json(cameraParam_path);

    cameraParam camera_param;
    camera_param.load_cameraparam(cameraParam_path);

    PSDDataset dataset;
    dataset.load_vision_psd("/home/chen/mydata/slot.json");
    dataset.load_loc_v2(loc_path);
    dataset.transform_vision_frame_to_world();
    dataset.draw_vision_slot(vision_psd_path);

    auto clusters = PSDProcessor::cluster_parking_slots(dataset.psd_frame_vision_vector, 0.4);
    PSDProcessor::merge_cluster(clusters);
    auto clusters_raw = clusters;
    PSDProcessor::draw_cluster(clusters, fusion_psd_path);
    PSDOptimizer::process(clusters, camera_param, hardware_config);
    PSDProcessor::draw_cluster(clusters, optimized_psd_path);


    //see result****************************
    std::string image_root = "/home/chen/mydata/bev/";

    std::string image_project = image_root + bag_path + "/";
    std::vector<std::string> image_names;
    Util::get_filenames(image_project, image_names);

    std::vector<std::pair<std::int64_t, std::string>> pair_data;

    for(const auto& image_name: image_names)
    {

        auto result = Util::split_string(image_name, "/");
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

        std::cout<<file_name<<std::endl;

        pair_data.push_back({std::stoll(file_name), image_name});
    }
    std::sort(pair_data.begin(), pair_data.end(), PairCMP());



    int index = 0;
    for(const auto& pair: pair_data)
    {

        std::double_t ts = pair.first * 1e-6;

        gtsam::Pose3 pose;
        if(dataset.get_pose(ts, pose))
        {
            cv::Mat image = cv::imread(pair.second);
            int index2 = 0;
            for(const auto& cluster: clusters)
            {
                auto pts_raw = clusters_raw[index2].transform_to_body(pose);
                index2 = index2 + 1;
                auto pts = cluster.transform_to_body(pose);

                cv::line(image, cv::Point2f(pts_raw[0].x(), pts_raw[0].y()), cv::Point2f(pts_raw[1].x(), pts_raw[1].y()), cv::Scalar(0, 0, 255), 1.5);
                cv::line(image, cv::Point2f(pts_raw[1].x(), pts_raw[1].y()), cv::Point2f(pts_raw[2].x(), pts_raw[2].y()), cv::Scalar(0, 0, 255), 1.5);
                cv::line(image, cv::Point2f(pts_raw[2].x(), pts_raw[2].y()), cv::Point2f(pts_raw[3].x(), pts_raw[3].y()), cv::Scalar(0, 0, 255), 1.5);
                cv::line(image, cv::Point2f(pts_raw[3].x(), pts_raw[3].y()), cv::Point2f(pts_raw[0].x(), pts_raw[0].y()), cv::Scalar(0, 0, 255), 1.5);


                cv::line(image, cv::Point2f(pts[0].x(), pts[0].y()), cv::Point2f(pts[1].x(), pts[1].y()), cv::Scalar(0, 255, 0), 1.5);
                cv::line(image, cv::Point2f(pts[1].x(), pts[1].y()), cv::Point2f(pts[2].x(), pts[2].y()), cv::Scalar(0, 255, 0), 1.5);
                cv::line(image, cv::Point2f(pts[2].x(), pts[2].y()), cv::Point2f(pts[3].x(), pts[3].y()), cv::Scalar(0, 255, 0), 1.5);
                cv::line(image, cv::Point2f(pts[3].x(), pts[3].y()), cv::Point2f(pts[0].x(), pts[0].y()), cv::Scalar(0, 255, 0), 1.5);

            }
            cv::Mat image_test = image(cv::Rect(0, 0, 1200, 1200));
            std::string image_index = image_project + "opt_parral/" + to_string(index) + ".jpg";
            index = index+1;
            cv::imwrite(image_index, image_test);
            cv::namedWindow("test", cv::WINDOW_NORMAL);
            cv::imshow("test", image_test);
            cv::waitKey(0);
        }

    }


    return 0;
}
