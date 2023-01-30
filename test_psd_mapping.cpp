#include <iostream>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include "scan_to_match.h"
#include "include/dataset.h"
#include<boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "gtsam/iou.h"
#include <unordered_set>
#include "vision_parking_slot_processor.h"
namespace fs = boost::filesystem;

int get_filenames(const std::string& dir, std::vector<std::string>& filenames)
{
    fs::path path(dir);
    if (!fs::exists(path))
    {
        return -1;
    }

    fs::directory_iterator end_iter;
    for (fs::directory_iterator iter(path); iter!=end_iter; ++iter)
    {
        if (fs::is_regular_file(iter->status()))
        {
            filenames.push_back(iter->path().string());
        }

        if (fs::is_directory(iter->status()))
        {
            get_filenames(iter->path().string(), filenames);
        }
    }

    return filenames.size();
}

std::vector<std::string> split_string(const std::string& ss, const std::string& token)
{
    std::vector<std::string>  tvs;
    boost::split(tvs, ss,boost::is_any_of(token),boost::token_compress_on);

    return tvs;
}

Json::Value load_json(const std::string& file_name)
{

    std::ifstream ifile;
    ifile.open(file_name);

    Json::CharReaderBuilder ReaderBuilder;
    ReaderBuilder["emitUTF8"] = true;//utf8支持，不加这句，utf8的中文字符会编程\uxxx
    Json::Value value;

    std::string strerr;
    if(!Json::parseFromStream(ReaderBuilder,ifile,&value,&strerr))
    {
        std::cerr<<"load json map"<<file_name<<" failed!"<<std::endl;
        exit(0);
    }

    return value;
}

struct PSD
{
    int id;
    double ts;
    std::vector<Eigen::Vector3d> pts;
    std::vector<bool> visible;

    IOU::Vertexes get_vertexes() const
    {
        IOU::Vertexes vertexes;
        for(const auto& p: pts)
        {
            vertexes.push_back(IOU::Vec2d(p.x(), p.y()));
        }

        return vertexes;
    }

    std::double_t get_area() const
    {
        return IOU::areaEx(get_vertexes());
    }
};

struct PSDFrame
{
    double ts;
    std::vector<PSD> psds;
};

struct Pose
{
    double ts;
    Eigen::Isometry3d data;
};

struct PoseCMP
{
    bool operator ()(const Pose& a, const Pose& b)
    {
        return a.ts < b.ts;
    }
};

struct PSDCMP
{
    bool operator ()(const PSDFrame& a, const PSDFrame& b)
    {
        return a.ts < b.ts;
    }
};

Pose load_pose(const std::string& file_name)
{
    auto value = load_json(file_name);
    Pose pose;
    pose.ts = value["meta"]["timestamp_us"].asInt64()*1e-6;
    pose.data.linear() = Eigen::Quaterniond(value["orientation"]["quaternion_local"]["w"].asDouble(),
                                            value["orientation"]["quaternion_local"]["x"].asDouble(),
                                            value["orientation"]["quaternion_local"]["y"].asDouble(),
                                            value["orientation"]["quaternion_local"]["z"].asDouble()).toRotationMatrix();
    pose.data.translation() = Eigen::Vector3d(value["position"]["position_local"]["x"].asDouble(),
                                              value["position"]["position_local"]["y"].asDouble(),
                                              value["position"]["position_local"]["z"].asDouble());
    return pose;
}

PSDFrame load_psds_vision(const std::string& file_name)
{
    const double distance_from_rear_axle_mid_to_vehicle_mid = 1.393;
    auto value = load_json(file_name);
    PSDFrame psd_frame;
    psd_frame.ts = value["meta"]["sensor_timestamp_us"].asInt64()*1e-6;
    for(int i = 0; i < value["parking_slots"].size(); ++i)
    {

        auto v = value["parking_slots"][i];
        if(v["corner_points"].size()  != 4)
        {
            continue;
        }
        PSD psd;
        psd.id = v["track_id"].asInt();
        int num_visible = 0;
        for(int j = 0; j < v["corner_points"].size(); ++j)
        {
            auto pts = v["corner_points"][j]["position"];
            double x = 0.02*(-pts["y"].asDouble() + 850.0/2.0) + distance_from_rear_axle_mid_to_vehicle_mid;
            double y = 0.02*(-pts["x"].asDouble() + 850.0/2.0);
            psd.pts.push_back(Eigen::Vector3d{x, y, 0});
            bool visible = v["corner_points"][j]["type"]["value"].asInt() == 1;
            if(visible)
            {
                num_visible++;
            }
            psd.visible.push_back(visible);
        }

        IOU::Vertexes vertexes;
        for(const auto& p: psd.pts)
        {
            vertexes.push_back(IOU::Vec2d(p.x(), p.y()));
        }

        std::double_t area = IOU::areaEx(vertexes);

        std::cout<<area<<std::endl;

        if(area / 10.0 < 0.6)
        {
            continue;
        }

        if(num_visible < 2)
        {
            continue;
        }

        int visual_index = -1;
        double distance = 0;
        for(int k=0;k<psd.pts.size();k++)
        {
            int next_idx = (k+1)%psd.pts.size();
            if(psd.visible[k] && psd.visible[next_idx])
            {
                visual_index = i;
                distance = (psd.pts[next_idx]-psd.pts[k]).norm();
            }
        }

        if(visual_index < 0)
        {
            continue;
        }
        int index1 = visual_index;
        int index2 = (visual_index+1)%psd.pts.size();
        int index3 = (visual_index+2)%psd.pts.size();
        int index4 = (visual_index+3)%psd.pts.size();

        psd_frame.psds.push_back(psd);
    }

    return psd_frame;
}

PSDFrame load_psds(const std::string& file_name)
{
    auto value = load_json(file_name);
    PSDFrame psd_frame;
    psd_frame.ts = value["meta"]["sensor_timestamp_us"].asInt64()*1e-6;
    for(int i = 0; i < value["parking_slot"]["parking_slot_data"].size(); ++i)
    {
        auto v = value["parking_slot"]["parking_slot_data"][i];
        PSD psd;
        psd.id = v["track_id"].asInt();
        for(int j = 0; j < v["local_points_fusion"].size(); ++j)
        {
            auto pts = v["local_points_fusion"][j];
            psd.pts.push_back(Eigen::Vector3d{pts["x"].asDouble(), pts["y"].asDouble(), pts["z"].asDouble()});
        }

        psd_frame.psds.push_back(psd);
    }

    return psd_frame;
}

void draw_line(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
               double interval, pcl::PointCloud<pcl::PointXYZRGB>& cloud,
               std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
    Eigen::Vector3d v = end - start;
    int size = std::ceil(v.norm() / interval);
    Eigen::Vector3d intv = v / size;
    for(int j = 0; j < size; ++j)
    {
        Eigen::Vector3d p_eigen = start + intv*j;
        pcl::PointXYZRGB psd_pt;
        psd_pt.x = p_eigen.x(); psd_pt.y = p_eigen.y(); psd_pt.z = p_eigen.z();
        psd_pt.r = r; psd_pt.g = g; psd_pt.b = b;
        cloud.push_back(psd_pt);
    }
    pcl::PointXYZRGB psd_pt;
    psd_pt.x = end.x(); psd_pt.y = end.y(); psd_pt.z = end.z();
    psd_pt.r = r; psd_pt.g = g; psd_pt.b = b;
    cloud.push_back(psd_pt);
}

void draw_lines(const std::vector<Eigen::Vector3d>& pts, double interval,
                pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                std::uint8_t r = 255, std::uint8_t g = 0, std::uint8_t b = 0)
{
    for(int i = 0; i < pts.size()-1; ++i)
    {
        draw_line(pts[i], pts[i+1], interval, cloud, r, g, b);
    }
    draw_line(pts.back(), pts[0], interval, cloud, r, g, b);
}

Eigen::Isometry3d interpolate_pose1(const Eigen::Isometry3d& left,
                                    std::double_t ts0,
                                    const Eigen::Isometry3d& right,
                                    std::double_t ts1,
                                    std::double_t ts)
{
    const double range_t = ts1 - ts0;
    const double ratio = (ts - ts0)/ range_t;
    Eigen::Quaterniond q1 = Eigen::Quaterniond (left.rotation());
    Eigen::Quaterniond q2 = Eigen::Quaterniond (right.rotation());
    Eigen::Vector3d t1 = left.translation();
    Eigen::Vector3d t2 = right.translation();

    Eigen::Isometry3d pose_out;
    pose_out.setIdentity();
    ///bilinear interpolation
    pose_out.translation() = (1.0 - ratio) * t1 + ratio * t2;
    pose_out.linear()      = q1.slerp(ratio, q2).toRotationMatrix();

    return pose_out;
}


class PointLineFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>
{
public:
    using Base     = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>;

    PointLineFactor(gtsam::Key const& pose_key,
                    gtsam::Key const& point_key,
                    gtsam::Point3 const& pt1,
                    gtsam::Point3 const& pt2,
                    gtsam::SharedNoiseModel const& noise_model): Base(noise_model, pose_key, point_key)
    {
        pt_        = pt1;
        direction_ = (pt1 - pt2).normalized();
    }

    virtual ~PointLineFactor() noexcept{};

    virtual gtsam::Vector evaluateError(gtsam::Pose3 const&             pose,
                                        gtsam::Point3 const&            pt,
                                        boost::optional<gtsam::Matrix&> H1 = boost::none,
                                        boost::optional<gtsam::Matrix&> H2 = boost::none) const override
    {
        gtsam::Matrix36 Hx1_pose;
        gtsam::Matrix33 Hx1_point;

        gtsam::Point3 x1 = pose.transform_from(pt, Hx1_pose, Hx1_point);

        // line direction
        gtsam::Point3 l1 = direction_;
        gtsam::Point3 l2 = x1 - pt_;

        gtsam::Matrix33 Hc_l2;

        gtsam::Point3   c = gtsam::cross(l2, l1, Hc_l2);
        gtsam::Matrix13 Ha_c;
        double          error = gtsam::norm3(c, Ha_c);

        if (H1)
        {
            *H1 = Ha_c * Hc_l2 * Hx1_pose;
        }

        if (H2)
        {
            *H2 = -Ha_c * Hc_l2 * Hx1_point;
        }

        return gtsam::Vector1(error);
    }

private:
    gtsam::Point3  pt_;  ///< source point eg. point in body frame
    gtsam::Vector3 direction_;  ///< the target line direction

};  // Point2LineFactor

struct PSDAssociation
{
    PSD psd_world_frame;
    std::vector<PSD> psd_body_observation;
};

std::vector<std::vector<PSD>> cluster_psds(const std::vector<PSD>& psds)
{
    std::vector<std::vector<PSD>> result;
    std::unordered_set<int> us;
    for(int i = 0; i < psds.size(); ++i)
    {
        auto vertexes_i = psds[i].get_vertexes();
        std::vector<PSD> cluster;
        cluster.push_back(psds[i]);
        std::double_t max_area = psds[i].get_area();
        std::int32_t  max_idx  = i;
        IOU::Vertexes max_vertexes;
        for(int j = i+1; j < psds.size(); ++j)
        {
            if(us.count(j))
            {
                continue;
            }
            auto vertexes_j = psds[j].get_vertexes();

            auto iou = IOU::iouEx(vertexes_i, vertexes_j);
            if(iou > 0.3)
            {
                us.insert(j);
                cluster.push_back(psds[j]);
                if(psds[j].get_area() > max_area)
                {
                    max_area = psds[j].get_area();
                    max_idx = j;
                    max_vertexes = psds[j].get_vertexes();
                }
            }
        }
        if(max_idx != i)
        {
            for(int j = i+1; j < psds.size(); ++j)
            {
                if(us.count(j))
                {
                    continue;
                }
                auto vertexes_j = psds[j].get_vertexes();

                auto iou = IOU::iouEx(max_vertexes, vertexes_j);
                if(iou > 0.3)
                {
                    us.insert(j);
                    cluster.push_back(psds[j]);
                }
            }
        }
        if(cluster.size() < 5)
        {
            continue;
        }
        result.push_back(cluster);
    }

    for(auto& cluster: result)
    {
        std::double_t max_area = 0;
        std::int32_t  max_idx  = 0;
        for(int i = 0; i < cluster.size(); ++i)
        {
            if(cluster[i].get_area() > max_area)
            {
                max_area = cluster[i].get_area();
                max_idx = i;
            }
        }
        std::swap(cluster[0], cluster[max_idx]);
    }

    return result;
}

using std::cout;
using std::endl;
using std::setprecision;

struct PSDCluster
{
    PSD psd_fusion;
    std::vector<PSD> psd_vision_vector;
};

int main()
{
    const std::string path = "/home/chenghe/Music/loc_slot/";
    std::vector<std::string> filenames;
    get_filenames(path, filenames);
    std::vector<PSDFrame> psd_frames;
    std::vector<PSDFrame> psd_frames_vision;
    std::vector<Pose> poses;
    pcl::PointCloud<pcl::PointXYZRGB> pts;
    pcl::PointCloud<pcl::PointXYZRGB> corner_pts;
    pcl::PointCloud<pcl::PointXYZRGB> fusion_pts;
    std::unordered_map<int, PSD> psd_fusion_um;

    for(const auto& f: filenames)
    {
        std::cout<<f<<std::endl;
        auto result = split_string(f, "/");
        if(result.empty())
        {
            continue;
        }
        auto file_name_split = split_string(result.back(), ".");
        if(file_name_split.size() != 2)
        {
            continue;
        }
        auto file_name = file_name_split.front();
        auto other_split = split_string(file_name, "_");
        if(other_split.size() < 2)
        {
            continue;
        }
        auto timestamp = std::stoll(other_split.front());
        auto type = other_split[1];
        std::cout<<"ts: "<<timestamp<<", type: "<<type<<std::endl;
        if(type == "fuison")
        {
            auto psd_frame = load_psds(f);
            psd_frames.push_back(psd_frame);
            for(const auto& psd: psd_frame.psds)
            {
                psd_fusion_um[psd.id] = psd;
            }
        }

        if(type == "vision")
        {
            auto psd_frame = load_psds_vision(f);
            psd_frames_vision.push_back(psd_frame);
        }

        if(type == "loc")
        {
            auto pose = load_pose(f);
            poses.push_back(pose);
            pcl::PointXYZRGB psd_pt;
            psd_pt.x = pose.data.translation().x(); psd_pt.y = pose.data.translation().y(); psd_pt.z = pose.data.translation().z();
            psd_pt.r = 0; psd_pt.g = 255; psd_pt.b = 0;
            pts.push_back(psd_pt);
        }
    }
    std::sort(poses.begin(), poses.end(), PoseCMP());
    std::sort(psd_frames_vision.begin(), psd_frames_vision.end(), PSDCMP());
    std::sort(psd_frames.begin(), psd_frames.end(), PSDCMP());
    std::vector<PSD> psds;
    for(auto& psd_vision: psd_frames_vision)
    {
        auto timestamp = psd_vision.ts;
        auto iter = poses.begin();
        auto next_iter = std::next(iter);
        bool found = false;
        Eigen::Isometry3d pose;
        for(; iter != poses.end() && next_iter != poses.end(); ++iter, ++next_iter)
        {
            if(timestamp >= iter->ts && timestamp <= next_iter->ts)
            {
                pose =  interpolate_pose1(iter->data, iter->ts, next_iter->data, next_iter->ts,
                                          timestamp);
                found = true;
                break;
            }
        }

        if(found)
        {
            for(auto& psd: psd_vision.psds)
            {
                for(auto& pt: psd.pts)
                {
                    Eigen::Vector3d pt_world = pose * pt;
                    pt = pt_world;
                }
                psds.push_back(psd);
            }

//            for(const auto& psd: psd_vision.psds)
//            {
//                draw_lines(psd.pts, 0.1, pts, 255, 0, 0);
//            }
        }
    }

    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<float> distr(0.0, 254.0);

    auto psd_cluster = cluster_psds(psds);

    for(const auto& cluster: psd_cluster)
    {
        std::uint8_t r = static_cast<std::uint8_t>( std::round(distr(eng)));
        std::uint8_t g = static_cast<std::uint8_t>( std::round(distr(eng)));
        std::uint8_t b = static_cast<std::uint8_t>( std::round(distr(eng)));

        PSDCluster psd_cluster;
        std::double_t max_iou = 0;
        for(const auto& pair: psd_fusion_um)
        {
            auto iou = IOU::iouEx(pair.second.get_vertexes(), cluster[0].get_vertexes());
            if(iou > max_iou)
            {
                psd_cluster.psd_fusion = pair.second;
                max_iou = iou;
            }
        }

        if(max_iou > 0.4)
        {
            std::double_t min_dist = std::numeric_limits<double>::max();
            int min_index = 0;
            int max_cnt = 0;
            for(int k = 0; k < 4; ++k)
            {
                auto ptf = psd_cluster.psd_fusion.pts[k];
                std::double_t dist_k = 0.0;
                int cnt = 1;
                for(int i = 0; i < cluster.size(); ++i)
                {
                    std::double_t min_dist_kk = std::numeric_limits<double>::max();
                    for(int kk = 0; kk < 4; ++kk)
                    {
                        auto ptv = cluster[i].pts[kk];
                        std::double_t dist = (ptf - ptv).norm();
                        if(dist < min_dist)
                        {
                            min_dist_kk = dist;
                        }
                    }
                    if(min_dist_kk < 2.0)
                    {
                        cnt++;
                        dist_k+=min_dist_kk;
                    }
                }
                std::cout<<"psd: point id: "<<k<<", distance: "<<dist_k/cnt<<", total: "<<dist_k
                         <<", size: "<<cnt<<std::endl;
                if(dist_k < min_dist)
                {
                    min_dist = dist_k;
                    min_index=k;
                }
            }
            std::cout<<min_dist/cluster.size()<<std::endl;

            pcl::PointXYZRGB p;
            p.x = psd_cluster.psd_fusion.pts[min_index].x();
            p.y = psd_cluster.psd_fusion.pts[min_index].y();
            p.z = psd_cluster.psd_fusion.pts[min_index].z();
            p.r = r; p.g = g; p.b = b;
            corner_pts.push_back(p);
            draw_lines(psd_cluster.psd_fusion.pts, 0.1, fusion_pts, r, g, b);
            for(const auto& psd: cluster)
            {
                draw_lines(psd.pts, 0.1, pts, r, g, b);
            }
        }
    }

    pcl::io::savePLYFile("/home/chenghe/psd.ply", pts);
    pcl::io::savePLYFile("/home/chenghe/psd_corner.ply", corner_pts);
    pcl::io::savePLYFile("/home/chenghe/psd_fusion.ply", fusion_pts);

    return 0;
}
