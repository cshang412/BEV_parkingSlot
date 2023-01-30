#ifndef HAOMO_STITCH_SDK_H_
#define HAOMO_STITCH_SDK_H_

#include <stdint.h>
#include <string>

namespace haomo {
namespace stitch {
    typedef enum Status {
        OK = 0,           // Operation succeeded.
        INVALID_ARGUMENT, // Invalid input argument.
        FAILURE           // Internal error.
    } Status;

    typedef enum DataType {
        DT_RGB = 0,
        DT_YUYV,
        DT_NV12,
        DT_NV16,
    } DataType;

    typedef struct CameraParam {
        double k2_, k3_, k4_, k5_;
        double fx_, fy_, cx_, cy_;
        double camera_tvec_[3];
        double camera_qvec_[4];
        int image_width_;
        int image_height_;
    } CameraParam;

    typedef struct CarParam {
        double car_length_;          // 车体长度
        double car_width_;           // 车体宽度
        double rearshaft2carcenter_; // 后轴中点到车体中心距离
        double wheel_diameter;
    } CarParam;

    typedef struct StitchParam {
        bool use_car_rect_;
        int image_width_;
        int image_height_;
        int extend_pix_;
        float pix_resolution_;
        float lean_angle_;
        std::string stitch_version_;
        std::string dewarp_type_;
        std::string reserved_;
    } StitchParam;

    typedef struct LutInfo {
        uint8_t *lut_buffer;      // LUT查找表buffer的首地址
        uint32_t lut_buffer_size; // LUT查找表buffer的大小
    } LutInfo;

    struct BlankArea {
        int blank_x_;
        int blank_y_;
        int blank_width_;
        int blank_height_;
        int stitch_width_;
        int stitch_height_;
    };

    Status IPM_generate_lut(void *camera_params[4], CarParam &car_param, StitchParam *ptr_stitch_param,
                            LutInfo *lut_info, DataType data_type);

    Status IPM_stitch(uint8_t *input_frame[4], LutInfo *lut_info, uint8_t *stitch_result);

    Status IPM_release_lut(LutInfo *lut_info);

    Status IPM_get_blank_mask(BlankArea &blank_area, uint8_t **blank_mask = nullptr);
}
} // namespace haomo

#endif