//
// Created by chenghe on 11/18/22.
//
#include "psd_optimizer.h"
#include "psd_factor.h"

namespace psd_mapping
{

    inline bool intersectJudge(std::vector<ParkingSlotPoint3> src, std::vector<ParkingSlotPoint3> des)
    {
        std::vector<cv::Point> contours;
        for(int i=0; i<4; i++){
            contours.push_back(cv::Point(des[i].position.x()/0.02, des[i].position.y()/0.02));
        }

        std::vector<cv::Point> pts_mid;
        for(int i=0; i<4; i++){
            int next_pt = (i+1)%4;
            pts_mid.push_back(cv::Point((src[i].position.x() + src[next_pt].position.x())/(0.02*2.0), (src[i].position.y() + src[next_pt].position.y())/(0.02*2.0)));
        }

        if(cv::pointPolygonTest(contours, pts_mid[0], false)==1 || cv::pointPolygonTest(contours, pts_mid[1], false)==1 ||
            cv::pointPolygonTest(contours, pts_mid[2], false)==1 || cv::pointPolygonTest(contours, pts_mid[3], false)==1)
        {
            return true;
        }
        else{
            return false;
        }
    }

    //from smaller to larger
    bool comp(const pair<int, double>a, const pair<int, double>b)
    {
        return a.second<b.second;
    }


    bool PSDOptimizer::process(std::vector<PSDAssociation>& clusters, cameraParam camera_param, HardwareConfig hardware_config)
    {
        ceres::Problem problem;
        ceres::LossFunction* loss_function = new ceres::CauchyLoss(0.05);
        ceres::LossFunction* loss_function1 = new ceres::CauchyLoss(std::sin(3/180.0*M_PI));

        for(auto& cluster: clusters)
        {
            cluster.init_data_array();
        }

        for(int i = 0; i < clusters.size(); ++i)
        {
            for(int j = i+1; j < clusters.size(); ++j)
            {
                bool BoxIntersect = false;
                BoxIntersect = intersectJudge(clusters[i].fusion_points_world, clusters[j].fusion_points_world);
                std::vector<std::pair<int, int>> pairs;
                for(int k = 0; k < 4; ++k)
                {
                    int index_closePt = -1;
                    double distance_min = 0.2;  

                    for(int z = 0; z < 4; ++z)
                    {

                        auto distance = (clusters[i].fusion_points_world[k].position -
                                clusters[j].fusion_points_world[z].position).norm();

                        if(distance < distance_min){
                            distance_min = distance;
                            index_closePt = z;
                        }

                        if(distance < 1.2)
                        {
                            pairs.push_back({k, z});
                            if(distance < 0.3)
                            {
                                //如果车位角点距离<0.3,优化平行方向距离=0
                                auto cost_function = PointFactor1::create(pairs.back().first, pairs.back().second);
                                problem.AddResidualBlock(cost_function, nullptr,
                                                         &clusters[i].p1_data.front(),
                                                         &clusters[i].d12_data.front(),
                                                         &clusters[i].d23_data.front(),
                                                         &clusters[j].p1_data.front(),
                                                         &clusters[j].d12_data.front(),
                                                         &clusters[j].d23_data.front());
                            }
                        }
                    }

                    if(BoxIntersect && index_closePt!=-1)
                    {       cout<<"have interset point"<<endl;
                            auto cost_function = PointFactor2::create(k, index_closePt);
                            problem.AddResidualBlock(cost_function, nullptr,
                                                    &clusters[i].p1_data.front(),
                                                    &clusters[i].d12_data.front(),
                                                    &clusters[i].d23_data.front(),
                                                    &clusters[j].p1_data.front(),
                                                    &clusters[j].d12_data.front(),
                                                    &clusters[j].d23_data.front());
                    }
                }


                if(!pairs.empty())
                {
                    auto dir12_i = clusters[i].get_dir12();
                    auto dir23_i = clusters[i].get_dir23();

                    auto dir12_j = clusters[j].get_dir12();
                    auto dir23_j = clusters[j].get_dir23();

                    {
                        auto angle = get_angle(dir12_i, dir12_j);
                        if(std::abs(angle)/M_PI*180 < 10.0)
                        {
                            std::cout<<"add edge2: "<<i<<", "<<j<<". "<<angle/M_PI*180.0<<std::endl;
                            auto cost_function  = EdgeFactor2::create();
                            problem.AddResidualBlock(cost_function, nullptr,
                                                     &clusters[i].d12_data.front(),
                                                     &clusters[j].d12_data.front());
                        }
                    }

                    {
                        auto angle = get_angle(dir12_i, dir23_j);
                        if(std::abs(angle)/M_PI*180 < 10.0)
                        {
                            std::cout<<"add edge2: "<<i<<", "<<j<<". "<<angle/M_PI*180.0<<std::endl;
                            auto cost_function  = EdgeFactor2::create();
                            problem.AddResidualBlock(cost_function, nullptr,
                                                     &clusters[i].d12_data.front(),
                                                     &clusters[j].d23_data.front());
                        }
                    }

                    {
                        auto angle = get_angle(dir23_i, dir12_j);
                        if(std::abs(angle)/M_PI*180 < 10.0)
                        {
                            std::cout<<"add edge2: "<<i<<", "<<j<<". "<<angle/M_PI*180.0<<std::endl;
                            auto cost_function  = EdgeFactor2::create();
                            problem.AddResidualBlock(cost_function, nullptr,
                                                     &clusters[i].d23_data.front(),
                                                     &clusters[j].d12_data.front());
                        }
                    }

                    {
                        auto angle = get_angle(dir23_i, dir23_j);
                        if(std::abs(angle)/M_PI*180 < 10.0)
                        {
                            std::cout<<"add edge2: "<<i<<", "<<j<<". "<<angle/M_PI*180.0<<std::endl;
                            auto cost_function  = EdgeFactor2::create();
                            problem.AddResidualBlock(cost_function, nullptr,
                                                     &clusters[i].d23_data.front(),
                                                     &clusters[j].d23_data.front());
                        }
                    }
                }
            }
        }

        for(int i = 0; i < clusters.size(); ++i)
        {
            vector<int> optslot_index;
            double ThHold = 10;
            vector<pair<int, double>> len_recorder;
            for(int j = 0; j < clusters[i].slots.size(); ++j)
            {
                bool flag = false;
                double min_len = DBL_MAX;
                for(int k=0; k<4; k++){
                    int next_pt = (k+1)%4;
                    if(clusters[i].slots[j].psd_point_image[k].confidence>0.5 && clusters[i].slots[j].psd_point_image[next_pt].confidence>0.5){
                        if((clusters[i].slots[j].psd_point_image[k].position - clusters[i].slots[j].psd_point_image[next_pt].position).norm()<min_len)
                        {
                            flag = true;
                            min_len = (clusters[i].slots[j].psd_point_image[k].position - clusters[i].slots[j].psd_point_image[next_pt].position).norm();
                        }
                    }
                }
                len_recorder.push_back(pair(j, min_len));
            }
            sort(len_recorder.begin(), len_recorder.end(), comp);
            int cut_index = len_recorder.size();
            for(int j=1; j<len_recorder.size(); j++){
                if(len_recorder[j].second-len_recorder[j-1].second>ThHold)
                    cut_index = j;
            }
            for(int j=0; j<cut_index; j++){
                optslot_index.push_back(len_recorder[j].first);
            }

            for(int j = 0; j < optslot_index.size(); ++j)
            {
                {
                    auto cost_function = PointFactor::create(clusters[i].slots[optslot_index[j]].psd_point_image,
                                                             clusters[i].slots[optslot_index[j]].pose,
                                                             camera_param,
                                                             hardware_config);
                    problem.AddResidualBlock(cost_function, loss_function,
                                                &clusters[i].p1_data.front(),
                                                &clusters[i].d12_data.front(),
                                                &clusters[i].d23_data.front());
                }

            }

            // 优化车位相邻边垂直
            auto cost_function  = EdgeFactor::create();
            problem.AddResidualBlock(cost_function, loss_function1,
                                     &clusters[i].d12_data.front(),
                                     &clusters[i].d23_data.front());
        }

        // solver options
        ceres::Solver::Options options;
        options.max_num_iterations = 100;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        // run the solver!
        ceres::Solve(options, &problem , &summary);

        for(auto& cluster: clusters)
        {
            cluster.update();
        }

        return true;
    }
}