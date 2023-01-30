//
// Created by chenghe on 8/3/22.
//

#ifndef CAMERA_MODEL_CYLINDER_CAMERA_MODEL_H
#define CAMERA_MODEL_CYLINDER_CAMERA_MODEL_H
#include "camera_model.h"
namespace haomo_cam
{
    class CylinderCamera: public BaseCamera
    {
    public:
        void init(const ConstPtr& origin_camera_ptr,
                  const CylinderParameters& cylinder_parameters);

        Eigen::Vector3d   image_to_unit_ray(const Eigen::Vector2d& pt_image) const override;

        Eigen::Vector3d   image_to_unit_plane(const Eigen::Vector2d& pt_image) const override;

        Eigen::Vector2d   camera_to_image(const Eigen::Vector3d& pt_cam) const override;

    private:
        CylinderParameters cylinder_parameters_;
    };
}


#endif //CAMERA_MODEL_CYLINDER_CAMERA_MODEL_H
