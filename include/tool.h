//
// Created by chenghe on 11/12/22.
//

#ifndef PSD_MAPPING_TOOL_H
#define PSD_MAPPING_TOOL_H

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include "psd_frame.h"
namespace psd_mapping
{
    class Tool
    {
    public:
        static void draw_line(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
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

        static void draw_slot(const std::vector<ParkingSlotPoint3>& pts, double interval,
                              pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                              std::uint8_t r = 255, std::uint8_t g = 0, std::uint8_t b = 0)
        {
            for(int i = 0; i < pts.size()-1; ++i)
            {
                draw_line(pts[i].position, pts[i+1].position, interval, cloud, r, g, b);
            }
            draw_line(pts.back().position, pts[0].position, interval, cloud, r, g, b);
        }
    };
}

#endif //PSD_MAPPING_TOOL_H
