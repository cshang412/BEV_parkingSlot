#include "camera.h"

using namespace std;
namespace haomo {
namespace stitch{
Size getImageSize(void *ptr_cam) {
    Size          image_size;
    CameraParam *ptr = (CameraParam *)ptr_cam;
    image_size.width = ptr->image_width_;
    image_size.height = ptr->image_height_;
    return image_size;
}

Status IPM_GenerateMap(MatrixF &map_f, MatrixUD &mask_ud, void *ptr_cam, Size &size, float pix_dis,
                       float rearshaft2carcenter, float wheel_diameter) {
    CameraParam *ptr = (CameraParam *)ptr_cam;
    return IPM_GenerateMap_(map_f, mask_ud, ptr, size, pix_dis, rearshaft2carcenter, wheel_diameter);
}
}
} // namespace haomo
