//
// Created by chenghe on 11/11/22.
//

#ifndef ODOMETER_PSD_FACTOR_H
#define ODOMETER_PSD_FACTOR_H

#include "vision_parking_slot_processor.h"
#include "ceres/ceres.h"
#include "psd_params.h"
#include "hardware_config.h"


namespace psd_mapping
{
    struct EdgeFactor
    {
        template<typename T>
        bool operator()(const T *const d1_ptr, const T *const d2_ptr, T *residual) const
        {
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dir1(d1_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dir2(d2_ptr);

            residual[0] = ceres::abs(dir1.normalized().dot(dir2.normalized()));


            return true;
        }

        static ceres::CostFunction* create()
        {
            ceres::AutoDiffCostFunction<EdgeFactor, 1, 3, 3>* cost_function = new
                    ceres::AutoDiffCostFunction<EdgeFactor, 1, 3, 3>(
                    new EdgeFactor());

            return cost_function;
        }
    };

    struct EdgeFactor2
    {
        template<typename T>
        bool operator()(const T *const d1_ptr, const T *const d2_ptr, T *residual) const
        {
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dir1(d1_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dir2(d2_ptr);

            residual[0] = 100.0*ceres::acos(ceres::abs(dir1.normalized().dot(dir2.normalized())));

            return true;
        }

        static ceres::CostFunction* create()
        {
            ceres::AutoDiffCostFunction<EdgeFactor2, 1, 3, 3>* cost_function = new
                    ceres::AutoDiffCostFunction<EdgeFactor2, 1, 3, 3>(
                    new EdgeFactor2());

            return cost_function;
        }
    };

    struct LineFactor
    {
        LineFactor(const std::vector<ParkingSlotPoint3>& _slot_points,
                   const int& _id):
                slot_points(_slot_points),
                id(_id)
        {

        }

        template<typename T>
        bool operator()(const T *const p_ptr, const T *const d1_ptr, const T *const d2_ptr, T *residual) const
        {
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p(p_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dir1(d1_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dir2(d2_ptr);

            Eigen::Matrix<T, 3, 1> point = p;

            if(id == 1)
            {
                point += dir1;
            }

            if(id == 2)
            {
                point += (dir1+dir2);
            }

            if(id == 3)
            {
                point += dir2;
            }

            int last_id = (id+3)%4;
            int next_id = (id+1)%4;

            residual[0] = (T)0.0;
            if(slot_points[id].visible || slot_points[last_id].visible)
            {
                Eigen::Vector3d dir = (slot_points[id].position - slot_points[last_id].position).normalized();
                Eigen::Matrix<T, 3, 1> distance = point - slot_points[id].position.template cast<T>();
                Eigen::Matrix<T, 3, 1> r = distance.template cross(dir.template cast<T>());
                residual[0] += r.norm();
            }

            if(slot_points[id].visible || slot_points[next_id].visible)
            {
                Eigen::Vector3d dir = (slot_points[id].position - slot_points[next_id].position).normalized();
                Eigen::Matrix<T, 3, 1> distance = point - slot_points[id].position.template cast<T>();
                Eigen::Matrix<T, 3, 1> r =  distance.template cross(dir.template cast<T>());
                residual[0] += r.norm();
            }

            return true;
        }

        static ceres::CostFunction* create(const std::vector<ParkingSlotPoint3>& _slot_points,
                                           const int& _id)
        {
            ceres::AutoDiffCostFunction<LineFactor, 1, 3, 3, 3>* cost_function = new
                    ceres::AutoDiffCostFunction<LineFactor, 1, 3, 3, 3>(
                    new LineFactor(_slot_points, _id));

            return cost_function;
        }

        std::vector<ParkingSlotPoint3> slot_points;
        int id;
    };

    struct PointFactor1
    {
        PointFactor1(int _id_a, int _id_b):
            id_a(_id_a),
            id_b(_id_b)
        {

        }

        template<typename T>
        bool operator()(const T *const pa_ptr, const T *const da1_ptr, const T *const da2_ptr,
                        const T *const pb_ptr, const T *const db1_ptr, const T *const db2_ptr,
                        T *residual) const
        {
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> pa(pa_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> pb(pb_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dira1(da1_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dira2(da2_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dirb1(db1_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dirb2(db2_ptr);

            Eigen::Matrix<T, 3, 1> pointa = pa;
            Eigen::Matrix<T, 3, 1> pointb = pb;
            Eigen::Matrix<T, 3, 1> direction = dirb1.norm() < dirb2.norm() ? dirb1: dirb2;


            if(id_a == 1)
            {
                pointa += dira1;
            }

            if(id_a == 2)
            {
                pointa += (dira1+dira2);
            }

            if(id_a == 3)
            {
                pointa += dira2;
            }

            if(id_b == 1)
            {
                pointb += dirb1;
            }

            if(id_b == 2)
            {
                pointb += (dirb1+dirb2);
            }

            if(id_b == 3)
            {
                pointb += dirb2;
            }

            // distance.template cross(dir.template cast<T>());
            const Eigen::Matrix<T, 3, 1> distance = direction.cross(pointa - pointb);

            residual[0] = 30.0 * distance.norm();
            // residual[1] = 30.0 * distance(1);
            // residual[2] = 30.0 * distance(2);

            return true;
        }

        static ceres::CostFunction* create(int _id_a, int _id_b)
        {
            ceres::AutoDiffCostFunction<PointFactor1, 1, 3, 3, 3, 3, 3, 3>* cost_function = new
                    ceres::AutoDiffCostFunction<PointFactor1, 1, 3, 3, 3, 3, 3, 3>(new PointFactor1(_id_a, _id_b));

            return cost_function;
        }

        int id_a;
        int id_b;
    };

    struct PointFactor2
    {
        PointFactor2(int _id_a, int _id_b):
            id_a(_id_a),
            id_b(_id_b)
        {

        }

        template<typename T>
        bool operator()(const T *const pa_ptr, const T *const da1_ptr, const T *const da2_ptr,
                        const T *const pb_ptr, const T *const db1_ptr, const T *const db2_ptr,
                        T *residual) const
        {
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> pa(pa_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> pb(pb_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dira1(da1_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dira2(da2_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dirb1(db1_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dirb2(db2_ptr);

            Eigen::Matrix<T, 3, 1> pointa = pa;
            Eigen::Matrix<T, 3, 1> pointb = pb;
            Eigen::Matrix<T, 3, 1> direction = dirb1.norm() > dirb2.norm() ? dirb1: dirb2;

            if(id_a == 1)
            {
                pointa += dira1;
            }

            if(id_a == 2)
            {
                pointa += (dira1+dira2);
            }

            if(id_a == 3)
            {
                pointa += dira2;
            }

            if(id_b == 1)
            {
                pointb += dirb1;
            }

            if(id_b == 2)
            {
                pointb += (dirb1+dirb2);
            }

            if(id_b == 3)
            {
                pointb += dirb2;
            }

            const Eigen::Matrix<T, 3, 1> distance = direction.cross(pointa - pointb);

            residual[0] = 30.0 * distance.norm();

            return true;
        }

        static ceres::CostFunction* create(int _id_a, int _id_b)
        {
            ceres::AutoDiffCostFunction<PointFactor2, 1, 3, 3, 3, 3, 3, 3>* cost_function = new
                    ceres::AutoDiffCostFunction<PointFactor2, 1, 3, 3, 3, 3, 3, 3>(new PointFactor2(_id_a, _id_b));

            return cost_function;
        }

        int id_a;
        int id_b;
    };


    struct EdgeFactor1
    {
        EdgeFactor1(int _id_a1, int _id_a2, int _id_b1, int _id_b2):
                id_a1(_id_a1),
                id_a2(_id_a2),
                id_b1(_id_b1),
                id_b2(_id_b2)

        {

        }

        template<typename T>
        bool operator()(const T *const pa_ptr, const T *const da1_ptr, const T *const da2_ptr,
                        const T *const pb_ptr, const T *const db1_ptr, const T *const db2_ptr,
                        T *residual) const
        {
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> pa(pa_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> pb(pb_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dira1(da1_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dira2(da2_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dirb1(db1_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dirb2(db2_ptr);

            Eigen::Matrix<T, 3, 1> pointa1 = pa;
            Eigen::Matrix<T, 3, 1> pointa2 = pa;
            Eigen::Matrix<T, 3, 1> pointb1 = pb;
            Eigen::Matrix<T, 3, 1> pointb2 = pb;

            if(id_a1 == 1)
            {
                pointa1 += dira1;
            }

            if(id_a1 == 2)
            {
                pointa1 += (dira1+dira2);
            }

            if(id_a1 == 3)
            {
                pointa1 += dira2;
            }

            if(id_a2 == 1)
            {
                pointa2 += dira1;
            }

            if(id_a2 == 2)
            {
                pointa2 += (dira1+dira2);
            }

            if(id_a2 == 3)
            {
                pointa2 += dira2;
            }

            if(id_b1 == 1)
            {
                pointb1 += dirb1;
            }

            if(id_b1 == 2)
            {
                pointb1 += (dirb1+dirb2);
            }

            if(id_b1 == 3)
            {
                pointb1 += dirb2;
            }

            if(id_b2 == 1)
            {
                pointb2 += dirb1;
            }

            if(id_b2 == 2)
            {
                pointb2 += (dirb1+dirb2);
            }

            if(id_b2 == 3)
            {
                pointb2 += dirb2;
            }

            Eigen::Matrix<T, 3, 1> dira12 = pointa2- pointa1;
            Eigen::Matrix<T, 3, 1> dirb12 = pointb2- pointb1;

            residual[0] = (1.0 - ceres::abs(dira12.normalized().dot(dirb12.normalized())));

            return true;
        }

        static ceres::CostFunction* create(int _id_a1, int _id_a2, int _id_b1, int _id_b2)
        {
            ceres::AutoDiffCostFunction<EdgeFactor1, 1, 3, 3, 3, 3, 3, 3>* cost_function = new
                    ceres::AutoDiffCostFunction<EdgeFactor1, 1, 3, 3, 3, 3, 3, 3>(new EdgeFactor1(_id_a1, _id_a2, _id_b1, _id_b2));

            return cost_function;
        }

        int id_a1;
        int id_a2;
        int id_b1;
        int id_b2;

    };

    struct PointFactor
    {
        PointFactor(const std::vector<ParkingSlotPoint2>& _slot_points,
                    const gtsam::Pose3& _pose,
                    const cameraParam _cam_param,
                    const HardwareConfig _hardware_config):
                    slot_points(_slot_points),
                    pose(_pose),
                    cam_param(_cam_param),
                    hardware_config(_hardware_config)
        {
        }

        template<typename T>
        bool operator()(const T *const p_ptr, const T *const d1_ptr, const T *const d2_ptr, T *residual) const
        {
            T error = T(DBL_MAX);
            const std::double_t pixel_size = PSDMappingParam::get_instance()->IPM_PIXEL_SIZE;
            const std::double_t stitch_size = PSDMappingParam::get_instance()->IPM_IMAGE_SIZE;
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p(p_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dir1(d1_ptr);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> dir2(d2_ptr);

            const Eigen::Matrix3d r_wb = pose.rotation().matrix();
            const Eigen::Vector3d t_wb = pose.translation();
            const Eigen::Matrix3d r_bw = r_wb.transpose();
            const Eigen::Vector3d t_bw = -r_bw * t_wb;

            vector<Eigen::Matrix<T, 3, 1>> pt_world;
            vector<Eigen::Matrix<T, 3, 1>> pt_body;
            vector<Eigen::Matrix<T, 2, 1>> pt_bev;;
            Eigen::Matrix<T, 3, 1> pt_unitPlane_fish;
            Eigen::Matrix<T, 3, 1> pt_unitPlane_bev;
            vector<Eigen::Matrix<T, 2, 1>> distance;

            pt_world.push_back(Eigen::Matrix<T, 3, 1>(p(0), p(1), p(2)));
            pt_world.push_back(Eigen::Matrix<T, 3, 1>(p(0)+dir1(0), p(1)+dir1(1), p(2)+dir1(2)));
            pt_world.push_back(Eigen::Matrix<T, 3, 1>(p(0)+dir1(0)+dir2(0), p(1)+dir1(1)+dir2(1), p(2)+dir1(2)+dir2(2)));
            pt_world.push_back(Eigen::Matrix<T, 3, 1>(p(0)+dir2(0), p(1)+dir2(1), p(2)+dir2(2)));

            for(int i=0; i<4; i++)
                pt_body.push_back(r_bw.template cast<T>() * pt_world[i] + t_bw.template cast<T>());
        
            for(int i=0; i<4; i++){
                if(pt_body[i](2)<-1.0 || slot_points[i].confidence<0.5){
                    distance.push_back(Eigen::Matrix<T, 2, 1>(0.0, 0.0));
                    continue;
                }

                auto cam_id = hardware_config.blank_mask.at<uchar>(int(slot_points[i].position.y()), int(slot_points[i].position.x()))/50-1;

                //body to camera to unit_camera
                Eigen::Matrix3d R_cb = cam_param.R[cam_id].transpose();
                Eigen::Vector3d T_cb = -R_cb * cam_param.T[cam_id];
                pt_unitPlane_fish = R_cb.template cast<T>() * pt_body[i] + T_cb.template cast<T>();
                pt_unitPlane_fish = pt_unitPlane_fish/pt_unitPlane_fish(2);

                //Homography to unit_bev
                pt_unitPlane_bev = cam_param.H[cam_id].template cast<T>() * pt_unitPlane_fish;
                pt_unitPlane_bev = pt_unitPlane_bev/pt_unitPlane_bev(2);

                //unit_bev to bev
                Eigen::Matrix<T, 3, 1> bev_temp;
                bev_temp = pt_unitPlane_bev * T(-cam_param.wheel_diameter*0.5);

                T y_temp = (-bev_temp.x() / 0.02) + 1200/2.0;
                T x_temp = (-bev_temp.y() / 0.02) + 1200/2.0;
                // int dd = int(x_temp);
                
                if(x_temp >= 0.0 && x_temp < 1200.0 && y_temp >= 0.0 && y_temp < 1200.0)
                {
                    pt_bev.push_back(Eigen::Matrix<T, 2, 1>(x_temp, y_temp));
                    distance.push_back(pt_bev.back() - slot_points[i].position.template cast<T>());
                }
            }

            residual[0] = distance[0](0);
            residual[1] = distance[0](1);
            residual[2] = distance[1](0);
            residual[3] = distance[1](1);
            residual[4] = distance[2](0);
            residual[5] = distance[2](1);
            residual[6] = distance[3](0);
            residual[7] = distance[3](1);

            return true;
        }

        static ceres::CostFunction* create(const std::vector<ParkingSlotPoint2>& _slot_points,
                                            const gtsam::Pose3& _pose,
                                            const cameraParam _cam_param,
                                            const HardwareConfig _hardware_config)
        {
            ceres::AutoDiffCostFunction<PointFactor, 8, 3, 3, 3>* cost_function = new
                    ceres::AutoDiffCostFunction<PointFactor, 8, 3, 3, 3>(
                    new PointFactor(_slot_points, _pose, _cam_param, _hardware_config));

            return cost_function;
        }

        std::vector<ParkingSlotPoint2> slot_points;
        gtsam::Pose3 pose;
        cameraParam cam_param;
        HardwareConfig hardware_config;

    };
}

#endif //ODOMETER_PSD_FACTOR_H