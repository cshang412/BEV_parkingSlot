//
// Created by chenghe on 11/11/22.
//

#ifndef ODOMETER_PSD_UTILS_H
#define ODOMETER_PSD_UTILS_H

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/inference/Symbol.h>

#include <json/json.h>
#include<boost/algorithm/string.hpp>
#include<boost/filesystem.hpp>
#include <unordered_set>
namespace fs = boost::filesystem;
namespace psd_mapping
{
    class Util
    {
    public:
        static int get_filenames(const std::string& dir, std::vector<std::string>& filenames)
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

        static std::vector<std::string> split_string(const std::string& ss, const std::string& token)
        {
            std::vector<std::string>  tvs;
            boost::split(tvs, ss,boost::is_any_of(token),boost::token_compress_on);

            return tvs;
        }

        static Json::Value load_json(const std::string& file_name)
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

            ifile.close();

            return value;
        }

        static gtsam::Pose3 interpolate_pose(const gtsam::Pose3& left,
                                             std::double_t ts0,
                                             const gtsam::Pose3& right,
                                             std::double_t ts1,
                                             std::double_t ts)
        {
            const double range_t = ts1 - ts0;
            const double ratio = (ts - ts0)/ range_t;
            Eigen::Quaterniond q1 = Eigen::Quaterniond (left.rotation().matrix());
            Eigen::Quaterniond q2 = Eigen::Quaterniond (right.rotation().matrix());
            Eigen::Vector3d t1 = left.translation();
            Eigen::Vector3d t2 = right.translation();

            gtsam::Vector3 t = (1.0 - ratio) * t1 + ratio * t2;
            gtsam::Rot3    r(q1.slerp(ratio, q2).toRotationMatrix());

            return gtsam::Pose3(r, t);
        }
    };
}

#endif //ODOMETER_PSD_UTILS_H
