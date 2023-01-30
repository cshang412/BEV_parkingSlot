#include "camera.h"
#include <Eigen/Geometry>
using namespace std;
namespace haomo {
namespace stitch {
static void IPM_GenerateRotation(MatrixD &rot, CameraParam *ptr) {
    auto   qvec = ptr->camera_qvec_;
    Eigen::Quaterniond q(qvec[0], qvec[1], qvec[2], qvec[3]);
    Eigen::Matrix3d r = q.toRotationMatrix();
    MatrixD c_i(3, 3, 1);
    c_i.setIdentity();
    rot = c_i;
    rot(0, 0) = r(0, 0);
    rot(0, 1) = r(0, 1);
    rot(0, 2) = r(0, 2);
    rot(1, 0) = r(1, 0);
    rot(1, 1) = r(1, 1);
    rot(1, 2) = r(1, 2);
    rot(2, 0) = r(2, 0);
    rot(2, 1) = r(2, 1);
    rot(2, 2) = r(2, 2);
}

static Pt2f IPM_Camera2UV(CameraParam *ptr, Pt3f &cp) {
    double dis = sqrt(cp.x * cp.x + cp.y * cp.y + cp.z * cp.z);

    double theta = acos(cp.z / dis);
    double phi = atan2(cp.y, cp.x);
    double theta2 = theta * theta;
    double theta4 = theta2 * theta2;
    double theta6 = theta4 * theta2;
    double theta8 = theta4 * theta4;
    double thetad = theta * (1 + ptr->k2_ * theta2 + ptr->k3_ * theta4 + ptr->k4_ * theta6 + ptr->k5_ * theta8);

    double u = ptr->fx_ * thetad * cos(phi) + ptr->cx_;
    double v = ptr->fy_ * thetad * sin(phi) + ptr->cy_;
    return Pt2f(u, v);
}

Status IPM_GenerateMap_(MatrixF &map_f, MatrixUD &mask_ud, CameraParam *ptr, Size &size,
                              float pix_dis, float rearshaft2carcenter, float wheel_diameter) {
    int mid_c = size.width / 2;
    int mid_r = size.height / 2 + (int)round(rearshaft2carcenter / pix_dis);

    map_f.set(size.height, size.width, 2);
    mask_ud.set(size.height, size.width, 1, 0);
    MatrixD rot_calib2cam;
    IPM_GenerateRotation(rot_calib2cam, ptr);

    for (int r = 0; r < size.height; r++) {
        for (int c = 0; c < size.width; c++) {
            float dx = -(r - mid_r) * pix_dis;
            float dy = -(c - mid_c) * pix_dis;
            float dz = -wheel_diameter*0.5;

            float px = dx - ptr->camera_tvec_[0];
            float py = dy - ptr->camera_tvec_[1];
            float pz = dz - ptr->camera_tvec_[2];

            float x = rot_calib2cam(0, 0) * px + rot_calib2cam(1, 0) * py + rot_calib2cam(2, 0) * pz;
            float y = rot_calib2cam(0, 1) * px + rot_calib2cam(1, 1) * py + rot_calib2cam(2, 1) * pz;
            float z = rot_calib2cam(0, 2) * px + rot_calib2cam(1, 2) * py + rot_calib2cam(2, 2) * pz;

            if (z > 0.0f) {
                Pt3f cp(x, y, z);
                Pt2f uv = IPM_Camera2UV(ptr, cp);
                bool check = (uv.x >= 0 && uv.y >= 0 && uv.x < ptr->image_width_ - 1 && uv.y < ptr->image_height_ - 1);
                if (check) {
                    mask_ud(r, c) = 255;
                    map_f(r, c, 0) = uv.x;
                    map_f(r, c, 1) = uv.y;
                }
            }
        }
    }
#ifdef DEBUG_OPENCV
    printMatrix(rot_calib2cam);
    Pt3f cp(1, 0, 1);
    Pt2f uv = IPM_Camera2UV(ptr, cp);
    MLOG_INFO("uv point %f %f", uv.x, uv.y);
#endif
    return Status::OK;
}
}
} // namespace haomo
