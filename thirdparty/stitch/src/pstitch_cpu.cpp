#include "camera.h"
#include "lut_info.h"
#include "matrix.hpp"
#include "stitch_interface.h"
#include <math.h>
#include <vector>

using namespace std;

namespace haomo {
namespace stitch{
typedef void (*remap_fun)(uint8_t *stitch_result, uint8_t *input_frame[4], vector<MatrixI> &vec_oper_map,
                          vector<MatrixUD> &vec_masks, vector<Pt2i> &vec_st_points, Size image_size,
                          Size output_size, vector<int> &vec_map_index, vector<int> &vec_camera_index);

void remapWithNV12Map(uint8_t *stitch_result, uint8_t *input_frame[4], vector<MatrixI> &vec_oper_map,
                      vector<MatrixUD> &vec_masks, vector<Pt2i> &vec_st_points, Size image_size,
                      Size output_size, vector<int> &vec_map_index, vector<int> &vec_camera_index) {
    int     map_num = vec_map_index.size();
    Pt2i tl_point(vec_st_points[0].x, vec_st_points[1].y);
    for (int i = 0; i < map_num; i++) {
        int mid = vec_map_index[i];
        int cid = vec_camera_index[i];

        Pt2i tl;
        tl.x = vec_st_points[mid].x - tl_point.x;
        tl.y = vec_st_points[mid].y - tl_point.y;
        Size mask_size;
        mask_size.width = vec_masks[mid].cols();
        mask_size.height = vec_masks[mid].rows();
        int uv_row = mask_size.height;
        for (int r = 0; r < mask_size.height; r++) {
            const uint8_t *ptr_mask = vec_masks[mid].ptr(r);
            uint8_t *      ptr_dst = stitch_result + (r + tl.y) * output_size.width;
            uint8_t *      ptr_uv = stitch_result + (int((r + tl.y) / 2) + output_size.height) * output_size.width;
            const int *    ptr_map = vec_oper_map[mid].ptr(r);
            const int *    ptr_map_uv = vec_oper_map[mid].ptr(uv_row);
            // std::cout << "mask_size.width:" << mask_size.width << std::endl;
            for (int c = 0; c < mask_size.width; c++) {
                if (ptr_mask[c] == 255) {
                    int y_idx = ptr_map[c];
                    *(ptr_dst + c + tl.x) = *(input_frame[cid] + y_idx);

                    if (r % 2 == 0) {
                        int uv_idx = ptr_map_uv[c];
                        // std::cout << "tl.x:" << tl.x << std::endl;
                        *(ptr_uv + c + tl.x) = *(input_frame[cid] + uv_idx);
                    }
                }
            }

            if (r % 2 == 1)
                uv_row++;
        }
    }
}

void remapWithNV16Map(uint8_t *stitch_result, uint8_t *ptr_src[4], vector<MatrixI> &vec_nv16_map,
                      vector<MatrixUD> &vec_masks, vector<Pt2i> &vec_st_points, Size image_size,
                      Size output_size, vector<int> &vec_map_index, vector<int> &vec_camera_index) {
    int     map_num = vec_map_index.size();
    int     dst_shift = output_size.width * output_size.height;
    Pt2i tl_point(vec_st_points[0].x, vec_st_points[1].y);
    for (int i = 0; i < map_num; i++) {
        int mid = vec_map_index[i];
        int cid = vec_camera_index[i];

        Pt2i tl;
        tl.x = vec_st_points[mid].x - tl_point.x;
        tl.y = vec_st_points[mid].y - tl_point.y;
        Size mask_size;
        mask_size.width = vec_masks[mid].cols();
        mask_size.height = vec_masks[mid].rows();
        for (int r = 0; r < mask_size.height; r++) {
            const uchar *ptr_mask = vec_masks[mid].ptr(r);
            uchar *      ptr_dst = stitch_result + (r + tl.y) * output_size.width;
            const int *  ptr_map = vec_nv16_map[mid].ptr(r);
            for (int c = 0; c < mask_size.width; c++) {
                if (ptr_mask[c] == 255) {
                    int rx = ptr_map[c * 2];
                    int ry = ptr_map[c * 2 + 1];
                    *(ptr_dst + c + tl.x) = *(ptr_src[cid] + rx);
                    *(ptr_dst + c + tl.x + dst_shift) = *(ptr_src[cid] + ry);
                }
            }
        }
    }
}

void remapWithYUYVMap(uint8_t *stitch_result, uint8_t *input_frame[4], vector<MatrixI> &vec_oper_maps,
                      vector<MatrixUD> &vec_masks, vector<Pt2i> &vec_st_points, Size image_size,
                      Size output_size, vector<int> &vec_map_index, vector<int> &vec_camera_index) {
    int     map_num = vec_map_index.size();
    Pt2i tl_point(vec_st_points[0].x, vec_st_points[1].y);
    for (int i = 0; i < map_num; i++) {
        int cid = vec_camera_index[i];
        int mid = vec_map_index[i];

        Pt2i tl;
        tl.x = vec_st_points[mid].x - tl_point.x;
        tl.y = vec_st_points[mid].y - tl_point.y;
        Size mask_size;
        mask_size.width = vec_masks[mid].cols();
        mask_size.height = vec_masks[mid].rows();
        for (int r = 0; r < mask_size.height; r++) {
            const uchar *ptr_mask = vec_masks[mid].ptr(r);
            uchar *      ptr_dst = stitch_result + (r + tl.y) * output_size.width * 2;
            const int *  ptr_map = vec_oper_maps[mid].ptr(r);
            for (int c = 0; c < mask_size.width; c++) {
                if (ptr_mask[c] == 255) {
                    int rx = ptr_map[c * 2];
                    int ry = ptr_map[c * 2 + 1];
                    *(ptr_dst + 2 * (c + tl.x)) = *(input_frame[cid] + rx);
                    *(ptr_dst + 2 * (c + tl.x) + 1) = *(input_frame[cid] + ry);
                }
            }
        }
    }
}

void remapWithBGRMap(uint8_t *stitch_result, uint8_t *input_frame[4], vector<MatrixI> &vec_oper_maps,
                     vector<MatrixUD> &vec_masks, vector<Pt2i> &vec_st_points, Size image_size,
                     Size output_size, vector<int> &vec_map_index, vector<int> &vec_camera_index) {
    int     map_num = vec_map_index.size();
    Pt2i tl_point(vec_st_points[0].x, vec_st_points[1].y);
    for (int i = 0; i < map_num; i++) {
        int     cid = vec_camera_index[i];
        int     mid = vec_map_index[i];
        Pt2i tl;
        tl.x = vec_st_points[mid].x - tl_point.x;
        tl.y = vec_st_points[mid].y - tl_point.y;
        Size mask_size;
        mask_size.width = vec_masks[mid].cols();
        mask_size.height = vec_masks[mid].rows();
        for (int r = 0; r < mask_size.height; r++) {
            const uchar *ptr_mask = vec_masks[mid].ptr(r);
            uchar *      ptr_dst = stitch_result + 3 * (r + tl.y) * output_size.width;
            const int *  ptr_map = vec_oper_maps[mid].ptr(r);
            for (int c = 0; c < mask_size.width; c++) {
                if (ptr_mask[c] == 255) {
                    int rx = ptr_map[c * 2];
                    int ry = ptr_map[c * 2 + 1];
                    ptr_dst[3 * (c + tl.x) + 0] = *(input_frame[cid] + (ry * image_size.width + rx) * 3);
                    ptr_dst[3 * (c + tl.x) + 1] = *(input_frame[cid] + (ry * image_size.width + rx) * 3 + 1);
                    ptr_dst[3 * (c + tl.x) + 2] = *(input_frame[cid] + (ry * image_size.width + rx) * 3 + 2);
                }
            }
        }
    }
}

Status IPM_stitch(uint8_t *input_frame[4], LutInfo *lut_info, uint8_t *stitch_result) {
    // 解析lut_info成lutinfoimp
    if (!g_init) {
        MLOG_ASSERT(Status::OK == lutInfo2LutImp(*lut_info, g_lut_imp));
        // lutInfo2LutImp(*lut_info, g_lut_imp);
        g_init = true;
    }

    vector<MatrixI> & vec_oper_map = g_lut_imp.vec_oper_map;
    vector<MatrixUD> &vec_sep_mask = g_lut_imp.vec_sep_mask;
    vector<Pt2i> &    vec_st_point = g_lut_imp.vec_st_point;
    Size &            image_size = g_lut_imp.image_size;
    Size &            output_size = g_lut_imp.stitch_size;

    vector<int> vec_map_index = {0, 1, 2, 3, 4, 5, 6, 7};
    vector<int> vec_camera_index = {3, 0, 1, 3, 1, 3, 2, 1};

    remap_fun ptr_fun = remapWithBGRMap;
    if (g_lut_imp.data_type == (int)DataType::DT_NV12)
        ptr_fun = remapWithNV12Map;
    else if (g_lut_imp.data_type == (int)DataType::DT_NV16)
        ptr_fun = remapWithNV16Map;
    else if (g_lut_imp.data_type == (int)DataType::DT_YUYV)
        ptr_fun = remapWithYUYVMap;

    (*ptr_fun)(stitch_result, input_frame, vec_oper_map, vec_sep_mask, vec_st_point, image_size, output_size,
               vec_map_index, vec_camera_index);

    return Status::OK;
}

void IPM_gen_blank_mask(vector<MatrixI> &vec_oper_maps) {
    vector<MatrixUD> &vec_masks = g_lut_imp.vec_sep_mask;
    vector<Pt2i> &    vec_st_points = g_lut_imp.vec_st_point;
    Size &            image_size = g_lut_imp.image_size;
    Size &            output_size = g_lut_imp.stitch_size;
    MatrixUD &        blank_mask = g_lut_imp.blank_mask;

    vector<int> vec_map_index = {0, 1, 2, 3, 4, 5, 6, 7};
    vector<int> vec_camera_index = {3, 0, 1, 3, 1, 3, 2, 1};

    MatrixUD front_image(image_size.height, image_size.width, 1, uchar(50));
    MatrixUD right_image(image_size.height, image_size.width, 1, uchar(100));
    MatrixUD rear_image(image_size.height, image_size.width, 1, uchar(150));
    MatrixUD left_image(image_size.height, image_size.width, 1, uchar(200));
    blank_mask.set(output_size.height, output_size.width);
    uint8_t* stitch_result = blank_mask.data();

    uint8_t* input_frame[4] = {front_image.data(), right_image.data(), rear_image.data(), left_image.data()};
    int     map_num = vec_map_index.size();
    Pt2i tl_point(vec_st_points[0].x, vec_st_points[1].y);
    for (int i = 0; i < map_num; i++) {
        int     cid = vec_camera_index[i];
        int     mid = vec_map_index[i];
        Pt2i tl;
        tl.x = vec_st_points[mid].x - tl_point.x;
        tl.y = vec_st_points[mid].y - tl_point.y;
        Size mask_size;
        mask_size.width = vec_masks[mid].cols();
        mask_size.height = vec_masks[mid].rows();
        for (int r = 0; r < mask_size.height; r++) {
            const uchar *ptr_mask = vec_masks[mid].ptr(r);
            uchar *      ptr_dst = stitch_result + (r + tl.y) * output_size.width;
            const int *  ptr_map = vec_oper_maps[mid].ptr(r);
            for (int c = 0; c < mask_size.width; c++) {
                if (ptr_mask[c] == 255) {
                    int rx = ptr_map[c * 2];
                    int ry = ptr_map[c * 2 + 1];
                    ptr_dst[c + tl.x] = *(input_frame[cid] + (ry * image_size.width + rx));
                }
            }
        }
    }
}
}
} // namespace haomo
