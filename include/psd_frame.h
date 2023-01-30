//
// Created by chenghe on 11/11/22.
//

#ifndef ODOMETER_PSD_FRAME_H
#define ODOMETER_PSD_FRAME_H
#include "psd_utils.h"
#include "psd_params.h"
#include "iou.h"
namespace psd_mapping
{
    enum class FrameType
    {
        PSD_FUSION,
        PSD_VISION,
        LOC,
    };

    class BaseFrame
    {
    public:
        std::double_t timestamp;
        virtual bool load_from_json(const std::string& file_name) = 0;
        virtual FrameType get_type() const = 0;
    };

    class LocFrame: public BaseFrame
    {
    public:
        gtsam::Key get_key() const;

        void set_pose(const Eigen::Quaterniond& q,
                      const Eigen::Vector3d&    t);

        FrameType get_type() const;

        bool load_from_json(const std::string& file_name) override;

        gtsam::Pose3 pose;
    };

    struct ParkingSlotPointBase
    {
        std::double_t        confidence;
        bool                 visible;
    };

    struct ParkingSlotPoint2: public ParkingSlotPointBase
    {
        ParkingSlotPoint2();

        gtsam::Point2 position;
    };

    struct ParkingSlotPoint3: public ParkingSlotPointBase
    {
        ParkingSlotPoint3();

        ParkingSlotPoint3& operator = (const ParkingSlotPoint2& other);

        void transform(const gtsam::Pose3& pose);

        gtsam::Point3 position;
    };

    class ParkingSlot
    {
    public:
        std::double_t timestamp;
        std::int32_t  track_id;
        std::double_t area;
        ParkingSlotPoint2 center_image;
        ParkingSlotPoint3 center_body;
        ParkingSlotPoint3 center_world;
        std::vector<ParkingSlotPoint2> psd_point_image;
        std::vector<ParkingSlotPoint3> psd_point_body;
        std::vector<ParkingSlotPoint3> psd_point_world;
        gtsam::Pose3 pose;

        IOU::Vertexes get_image_vertexes() const;

        IOU::Vertexes get_body_vertexes() const;

        IOU::Vertexes get_world_vertexes() const;

        void transform_to_world(const gtsam::Pose3& pose);

    private:

        template<class T>
        IOU::Vertexes get_vertexes(const T& t) const
        {
            IOU::Vertexes vertexes;
            for(const auto& p: t)
            {
                vertexes.push_back(IOU::Vec2d(p.position.x(), p.position.y()));
            }

            return vertexes;
        }
    };

    class ParkingSlotFrame: public BaseFrame
    {
    public:
        ParkingSlotFrame(FrameType type);

        void transform_to_world(const gtsam::Pose3& pose);

        bool load_from_json(const std::string& file_name) override;

        FrameType get_type() const override;

        std::vector<ParkingSlot> park_slots;

    private:

        bool load_vision_frame(const std::string& file_name);

        bool load_fusion_frame(const std::string& file_name);

        FrameType type_;
    };

    struct FrameCMP
    {
        bool operator ()(const BaseFrame& a, const BaseFrame& b)
        {
            return a.timestamp < b.timestamp;
        }
    };
}

#endif //ODOMETER_PSD_FRAME_H
