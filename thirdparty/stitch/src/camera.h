#ifndef STITCH_CAMERA_H_
#define STITCH_CAMERA_H_

#include "matrix.hpp"
#include "stitch_interface.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace haomo {
namespace stitch{
Size getImageSize(void *ptr_cam);

Status IPM_GenerateMap_(MatrixF &map_f, MatrixUD &mask_ud, CameraParam *ptr_cam, Size &size,
                        float pix_dis, float rearshaft2carcenter, float wheel_diameter);

Status IPM_GenerateMap(MatrixF &map_f, MatrixUD &mask_ud, void *ptr_cam, Size &size, float pix_dis,
                       float rearshaft2carcenter, float wheel_diameter);
}
} // namespace haomo

#endif
