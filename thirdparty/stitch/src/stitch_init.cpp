#include "camera.h"
#include "lut_info.h"
#include "matrix.hpp"
#include "stitch_interface.h"
#include <math.h>
#include <vector>

using namespace std;
namespace haomo {
    namespace stitch {
        LutInfoImp g_lut_imp;
        bool       g_init = false;

        Rect IPM_CalcInnerRect(vector<MatrixUD> &vec_init_mask, Size car_pix_size, int extend_off);

        void IPM_SeperateMask(vector<MatrixUD> &vec_init_mask, vector<MatrixUD> &vec_sep_mask,
                              vector<Pt2i> &vec_st_point, Size &output_size, Rect &use_rect, float lean_angle);

        void IPM_SeperateMap(vector<MatrixI> &vec_sep_map, vector<MatrixUD> &vec_sep_mask, vector<Pt2i> &vec_st_point,
                             vector<MatrixI> &vec_round_map, vector<MatrixUD> &tmp_sep_mask,
                             vector<Pt2i> &tmp_st_point);

        Status IPM_RoundMap(MatrixI& round_map, MatrixF& src_map);

        Status IPM_RoundMap(vector<MatrixI> &vec_round_map, vector<MatrixF> &vec_sep_map);

        Status IPM_RoundMap2NV12(vector<MatrixI> &vec_oper_map, vector<MatrixI> &vec_round_map,
                                 vector<MatrixUD> &vec_sep_mask, vector<Pt2i> &vec_st_point, Size &image_size,
                                 Size &stitch_size);

        Status IPM_RoundMap2NV16(vector<MatrixI> &vec_oper_map, vector<MatrixI> &vec_round_map,
                                 vector<MatrixUD> &vec_sep_mask, vector<Pt2i> &vec_st_point, Size &image_size,
                                 Size &stitch_size);

        Status IPM_RoundMap2YUYV(vector<MatrixI> &vec_oper_map, vector<MatrixI> &vec_round_map,
                                 vector<MatrixUD> &vec_sep_mask, vector<Pt2i> &vec_st_point, Size &image_size,
                                 Size &stitch_size);

        void    IPM_gen_blank_mask(vector<MatrixI> &vec_oper_map);

        void IPM_CheckMapAndMask(MatrixI& round_map, MatrixUD& mask, Size& image_size);


        template <typename T> void IPM_ReleaseMatrix(vector<Matrix<T>> &vec_matrix) {
            for (auto &matrix : vec_matrix)
                matrix.release();
            vec_matrix.clear();
        }

        template <typename T> void printMatrix(Matrix<T> &mat) {
            int step = 1;
            if (mat.rows() > 100)
                step = 20;
            // string format = "rows %d cols %d ch %d, %f\n";
            // if (typeid(T) == typeid(int))
            std::string format = "rows %d cols %d ch %d, %d\n";

            MLOG_INFO("print Matrix:");
            int print_count = 0;
            int max_count = 10;
            for (int r = 0; r < mat.rows(); r += step) {
                for (int c = 0; c < mat.cols(); c += step) {
                    for (int ch = 0; ch < mat.channels(); ch++) {
//                MLOG_INFO(format.c_str(), r, c, ch, mat(r, c, ch));
                        print_count++;
                        if (print_count > max_count)
                            return;
                    }
                }
            }
        }

//======================================================================================================================================================================

        Status IPM_generate_lut(void *camera_params[4], CarParam &car_param, StitchParam *ptr_stitch_param,
                                LutInfo *lut_info, DataType data_type) {
            if (g_init) {
                MLOG_WARN("lut info is existed, return\n");
                return Status::OK;
            }
            // 计算拼接图尺寸
            auto    stitch_width = ptr_stitch_param->image_width_;
            auto    stitch_height = ptr_stitch_param->image_height_;
            Size output_size = Size(stitch_width, stitch_height);
            MLOG_INFO("output stitch image size %dx%d\n", stitch_width, stitch_height);

            // 依据内外参生成拼接模板
            const int           camera_num = 4;
            vector<MatrixF>  vec_init_map(camera_num);
            vector<MatrixUD> vec_init_mask(camera_num);
            vector<MatrixI> vec_round_map(camera_num);
            for (int cid = 0; cid < camera_num; cid++) {
                IPM_GenerateMap(vec_init_map[cid], vec_init_mask[cid], camera_params[cid], output_size,
                                ptr_stitch_param->pix_resolution_, car_param.rearshaft2carcenter_, car_param.wheel_diameter);
                //#ifdef DEBUG_OPENCV
                //        saveMask(vec_init_mask[cid], std::to_string(cid) +
                //        "_init_mask.png"); printMatrix(vec_init_map[cid]);
                //#endif

                // need to recheck map and mask, after round operation!!!
                IPM_RoundMap(vec_round_map[cid], vec_init_map[cid]);

                Size            image_size = getImageSize(camera_params[cid]);
                IPM_CheckMapAndMask(vec_round_map[cid], vec_init_mask[cid], image_size);
                MLOG_INFO("check map and mask after round map for camera %d", cid);
            }

            // 计算车辆对应的黑框大小
            int     extend_pix = ptr_stitch_param->extend_pix_;
            int     car_pix_width = (int)round(car_param.car_width_ / ptr_stitch_param->pix_resolution_);
            int     car_pix_height = (int)round(car_param.car_length_ / ptr_stitch_param->pix_resolution_);
            Size car_pix_size(car_pix_width, car_pix_height);
            Rect car_rect(stitch_width / 2 - car_pix_width / 2 - extend_pix,
                          stitch_height / 2 - car_pix_height / 2 - extend_pix, car_pix_width + 2 * extend_pix,
                          car_pix_height + 2 * extend_pix);

            // 计算最终黑框大小
            Rect use_rect = car_rect;
            if (!ptr_stitch_param->use_car_rect_) {
                use_rect = IPM_CalcInnerRect(vec_init_mask, car_pix_size, extend_pix);
                MLOG_INFO("use auto rect %d %d %d %d\n", use_rect.x, use_rect.y, use_rect.width, use_rect.height);
            } else {
                MLOG_INFO("use car rect %d %d %d %d\n", use_rect.x, use_rect.y, use_rect.width, use_rect.height);
            }
            MLOG_ASSERT(use_rect.x >= 0 && use_rect.y >= 0 && use_rect.x + use_rect.width < output_size.width &&
                        use_rect.y + use_rect.height < output_size.height);

            g_lut_imp.blank_area.blank_x_ = use_rect.x;
            g_lut_imp.blank_area.blank_y_ = use_rect.y;
            g_lut_imp.blank_area.blank_width_ = use_rect.width;
            g_lut_imp.blank_area.blank_height_ = use_rect.height;
            g_lut_imp.blank_area.stitch_width_ = output_size.width;
            g_lut_imp.blank_area.stitch_height_ = output_size.height;

            // 将拼接模板进行分块
            vector<MatrixUD> tmp_sep_mask;
            vector<Pt2i>     tmp_st_point;
            IPM_SeperateMask(vec_init_mask, tmp_sep_mask, tmp_st_point, output_size, use_rect, ptr_stitch_param->lean_angle_);
            IPM_ReleaseMatrix(vec_init_mask);
            MLOG_INFO("seperate mask done!\n");

            // printMatrix(tmp_sep_mask[2]);

            vector<MatrixI>   vec_sep_map;
            vector<MatrixUD> &vec_sep_mask = g_lut_imp.vec_sep_mask;
            vector<Pt2i> &    vec_st_point = g_lut_imp.vec_st_point;
            IPM_SeperateMap(vec_sep_map, vec_sep_mask, vec_st_point, vec_round_map, tmp_sep_mask, tmp_st_point);
            IPM_ReleaseMatrix(vec_init_map);
            IPM_ReleaseMatrix(tmp_sep_mask);
            MLOG_INFO("seperate map done!\n");

            // printMatrix(vec_sep_mask[2]);

            // 将分块模板取整，做格式转换
            Size            image_size = getImageSize(camera_params[0]);
            IPM_ReleaseMatrix(vec_round_map);
            MLOG_INFO("round map done!\n");

            vector<MatrixI> &vec_oper_map = g_lut_imp.vec_oper_map;

            if (data_type == DataType::DT_NV12) {
                IPM_RoundMap2NV12(vec_oper_map, vec_sep_map, vec_sep_mask, vec_st_point, image_size, output_size);
                MLOG_INFO("round map to nv12!\n");
            } else if (data_type == DataType::DT_NV16) {
                IPM_RoundMap2NV16(vec_oper_map, vec_sep_map, vec_sep_mask, vec_st_point, image_size, output_size);
                MLOG_INFO("round map to nv16!\n");
            } else if (data_type == DataType::DT_YUYV) {
                IPM_RoundMap2YUYV(vec_oper_map, vec_sep_map, vec_sep_mask, vec_st_point, image_size, output_size);
                MLOG_INFO("round map to yuyv!\n");
            } else {
                vec_oper_map.resize(vec_sep_map.size());
                for (size_t i = 0; i < vec_sep_map.size(); i++)
                    vec_oper_map[i] = vec_sep_map[i];
                MLOG_INFO("use bgr map!\n");
            }

            g_lut_imp.image_size.width = image_size.width;
            g_lut_imp.image_size.height = image_size.height;
            g_lut_imp.stitch_size.width = output_size.width;
            g_lut_imp.stitch_size.height = output_size.height;
            g_lut_imp.data_type = data_type;

            // 计算拼接mask
            IPM_gen_blank_mask(vec_sep_map);
            MLOG_INFO("generate blank mask done!\n");
            MLOG_INFO("stitch init success!\n");
            g_init = true;

            IPM_ReleaseMatrix(vec_sep_map);

            return lutImp2LutInfo(g_lut_imp, *lut_info);
        }

/**********************************************************************************************************************************************************************************************/

        Rect IPM_CalcInnerRect(vector<MatrixUD> &vec_mask, Size car_pix_size, int extend_off) {
            MLOG_ASSERT(vec_mask.size() == 4);
            Size output_size = vec_mask[0].size();
            MLOG_ASSERT(output_size.height > 0 && output_size.width > 0);

            Rect     inner_rect;
            MatrixUD whole_mask(output_size.height, output_size.width);
            for (int r = 0; r < output_size.height; r++) {
                for (int c = 0; c < output_size.width; c++) {
                    bool check_pix = vec_mask[0](r, c) > 100 || vec_mask[1](r, c) > 100 || vec_mask[2](r, c) > 100 ||
                                     vec_mask[3](r, c) > 100;
                    if (check_pix)
                        whole_mask(r, c) = 255;
                }
            }

            // calc top y
            int minx = 0, maxx = 0, miny = 0, maxy = 0;
            for (int r = 0; r < whole_mask.rows(); r++) {
                const uchar *ptr = whole_mask.ptr(r);
                bool         jump = false;
                for (int c = 0; c < whole_mask.cols(); c++) {
                    if (ptr[c] == 0) {
                        miny = r;
                        jump = true;
                        break;
                    }
                }
                if (jump)
                    break;
            }

            if (miny < 0) {
                MLOG_ERROR("calc inner rect, miny map error!");
                return inner_rect;
            }

            // calc left x
            for (int c = 0; c < whole_mask.cols(); c++) {
                bool jump = false;
                for (int r = miny; r < whole_mask.rows(); r++) {
                    if (whole_mask(r, c) == 0) {
                        minx = c;
                        jump = true;
                        break;
                    }
                }
                if (jump)
                    break;
            }

            // calc bot y
            for (int r = whole_mask.rows() - 1; r > miny; r--) {
                const uchar *ptr = whole_mask.ptr(r);
                bool         jump = false;
                for (int c = 0; c < whole_mask.cols(); c++) {
                    if (ptr[c] == 0) {
                        maxy = r;
                        jump = true;
                        break;
                    }
                }
                if (jump)
                    break;
            }

            if (maxy > whole_mask.rows() - 1) {
                MLOG_ERROR("calc inner rect, maxy map error\n");
                return inner_rect;
            }

            // calc right x
            for (int c = whole_mask.cols() - 1; c > minx; c--) {
                bool jump = false;
                for (int r = 0; r < whole_mask.rows(); r++) {
                    if (whole_mask(r, c) == 0) {
                        maxx = c;
                        jump = true;
                        break;
                    }
                }
                if (jump)
                    break;
            }

            inner_rect = Rect(minx, miny, maxx - minx + 1, maxy - miny + 1);
            inner_rect.x -= extend_off;
            inner_rect.y -= extend_off;
            inner_rect.width += 2 * extend_off;
            inner_rect.height += 2 * extend_off;
            return inner_rect;
        }

        void IPM_SeperateMask(vector<MatrixUD> &vec_init_mask, vector<MatrixUD> &vec_sep_mask,
                              vector<Pt2i> &vec_st_point, Size &output_size, Rect &use_rect, float lean_angle) {
            if (lean_angle < 0.0f) {
                MLOG_INFO("lean angle set to 0.0f\n");
                lean_angle = 0.0f;
            }

            if (lean_angle > 89.9f) {
                MLOG_INFO("lean angle set to 90.0f!\n");
                lean_angle = 89.9f;
            }

            Rect outer_rect(0, 0, output_size.width, output_size.height);

            // check lean angle
            int     lean_pix = (int)round(fabs((use_rect.y) * tan(lean_angle / 180.0f * M_PI)));
            Pt2i tl(use_rect.x - lean_pix, 0);
            Pt2i tr(use_rect.br().x + lean_pix, 0);
            Pt2i bl(use_rect.x - lean_pix, outer_rect.br().y);
            Pt2i br(use_rect.br().x + lean_pix, outer_rect.br().y);
            if (lean_pix > use_rect.y) {
                lean_pix = (int)round(use_rect.x * tan(M_PI / 2 - lean_angle / 180.0f * M_PI));
                if (lean_pix < 0) {
                    MLOG_INFO("lean y %d is less than 0, set to 0\n", lean_pix);
                    lean_pix = 0;
                }

                tl = Pt2i(0, use_rect.y - lean_pix);
                tr = Pt2i(outer_rect.br().x, use_rect.y - lean_pix);
                bl = Pt2i(0, use_rect.br().y + lean_pix);
                br = Pt2i(outer_rect.br().x, use_rect.br().y + lean_pix);
            }

            int     left_minx = tl.x;
            int     left_maxx = use_rect.x;
            Pt2i temp(use_rect.br().x, use_rect.br().y);
            int     right_minx = temp.x;
            int     right_maxx = tr.x;
            int     top_y = use_rect.y;
            int     bot_y = temp.y;
            int     top_lr_y = tl.y;
            int     bot_lr_y = bl.y;

            int top_height = use_rect.y;
            int bot_height = outer_rect.br().y - use_rect.br().y;
            int left_width = left_maxx;
            int right_width = outer_rect.br().x - right_minx;
            int mid_height = bot_y - top_y + 1;
            int mid_width = right_maxx - left_minx + 1;
            int top_lr_height = use_rect.y - top_lr_y;
            int bot_lr_height = bot_lr_y - use_rect.br().y;

            vector<Rect> vec_sep_rects = {
                    Rect(0, top_lr_y, left_width, top_lr_height),               // tl rect
                    Rect(left_minx, 0, mid_width, top_height),                  // top-mid rect
                    Rect(right_minx + 1, top_lr_y, right_width, top_lr_height), // tr rect
                    Rect(0, top_y, left_width, mid_height),                     // left-mid rect
                    Rect(right_minx + 1, top_y, right_width, mid_height),       // right-mid rect
                    Rect(0, bot_y + 1, left_width, bot_lr_height),              // bl rect
                    Rect(left_minx, bot_y + 1, mid_width, bot_height),          // bot-mid rect
                    Rect(right_minx + 1, bot_y + 1, right_width, bot_lr_height) // br rect
            };

            auto line = [&](int x1, int y1, int x2, int y2, int x, int y) {
                return (y2 - y1) * (x - x1) - (x2 - x1) * (y - y1);
            };
            auto line1 = [&](int x, int y) { return line(tl.x, tl.y, use_rect.x, use_rect.y, x, y); };
            auto line2 = [&](int x, int y) { return line(tr.x, tr.y, use_rect.br().x, use_rect.y, x, y); };
            auto line3 = [&](int x, int y) { return line(use_rect.x, use_rect.br().y, bl.x, bl.y, x, y); };
            auto line4 = [&](int x, int y) { return line(use_rect.br().x, use_rect.br().y, br.x, br.y, x, y); };

            auto check_tl = [&](int x, int y) { return line1(x, y) > 0; };
            auto check_top = [&](int x, int y) { return (line1(x, y) < 0 || line2(x, y) > 0); };
            auto check_tr = [&](int x, int y) { return line2(x, y) < 0; };
            auto check_bl = [&](int x, int y) { return line3(x, y) > 0; };
            auto check_bot = [&](int x, int y) { return (line3(x, y) < 0 || line4(x, y) > 0); };
            auto check_br = [&](int x, int y) { return line4(x, y) < 0; };

            int rect_num = vec_sep_rects.size();
            vec_st_point.resize(rect_num);
            vec_sep_mask.resize(rect_num);
            for (int i = 0; i < 8; i++) {
                vec_sep_mask[i].set(vec_sep_rects[i].height, vec_sep_rects[i].width, 1, 255);
                vec_st_point[i] = vec_sep_rects[i].tl();
            }

            int id = 0;
            for (int r = 0; r < vec_sep_mask[id].rows(); r++) {
                for (int c = 0; c < vec_sep_mask[id].cols(); c++) {
                    int x = c + vec_st_point[id].x;
                    int y = r + vec_st_point[id].y;
                    // if (check_tl(x, y))
                    if (check_tl(x, y) || vec_init_mask[3](y, x) == 0)
                        vec_sep_mask[id](r, c) = 0;
                }
            }

            id = 1;
            for (int r = 0; r < vec_sep_mask[id].rows(); r++) {
                for (int c = 0; c < vec_sep_mask[id].cols(); c++) {
                    int x = c + vec_st_point[id].x;
                    int y = r + vec_st_point[id].y;
                    // if (check_top(x, y))
                    if (check_top(x, y) || vec_init_mask[0](y, x) == 0)
                        vec_sep_mask[id](r, c) = 0;
                }
            }

            id = 2;
            for (int r = 0; r < vec_sep_mask[id].rows(); r++) {
                for (int c = 0; c < vec_sep_mask[id].cols(); c++) {
                    int x = c + vec_st_point[id].x;
                    int y = r + vec_st_point[id].y;
                    // if (check_tr(x, y))
                    if (check_tr(x, y) || vec_init_mask[1](y, x) == 0)
                        vec_sep_mask[id](r, c) = 0;
                }
            }

            id = 3;
            for (int r = 0; r < vec_sep_mask[id].rows(); r++) {
                for (int c = 0; c < vec_sep_mask[id].cols(); c++) {
                    int x = c + vec_st_point[id].x;
                    int y = r + vec_st_point[id].y;
                    // if (check_bl(x, y))
                    if (vec_init_mask[3](y, x) == 0)
                        vec_sep_mask[id](r, c) = 0;
                }
            }

            id = 4;
            for (int r = 0; r < vec_sep_mask[id].rows(); r++) {
                for (int c = 0; c < vec_sep_mask[id].cols(); c++) {
                    int x = c + vec_st_point[id].x;
                    int y = r + vec_st_point[id].y;
                    // if (check_bl(x, y))
                    if (vec_init_mask[1](y, x) == 0)
                        vec_sep_mask[id](r, c) = 0;
                }
            }

            id = 5;
            for (int r = 0; r < vec_sep_mask[id].rows(); r++) {
                for (int c = 0; c < vec_sep_mask[id].cols(); c++) {
                    int x = c + vec_st_point[id].x;
                    int y = r + vec_st_point[id].y;
                    // if (check_bl(x, y))
                    if (check_bl(x, y) || vec_init_mask[3](y, x) == 0)
                        vec_sep_mask[id](r, c) = 0;
                }
            }

            id = 6;
            for (int r = 0; r < vec_sep_mask[id].rows(); r++) {
                for (int c = 0; c < vec_sep_mask[id].cols(); c++) {
                    int x = c + vec_st_point[id].x;
                    int y = r + vec_st_point[id].y;
                    // if (check_bot(x, y))
                    if (check_bot(x, y) || vec_init_mask[2](y, x) == 0)
                        vec_sep_mask[id](r, c) = 0;
                }
            }

            id = 7;
            for (int r = 0; r < vec_sep_mask[id].rows(); r++) {
                for (int c = 0; c < vec_sep_mask[id].cols(); c++) {
                    int x = c + vec_st_point[id].x;
                    int y = r + vec_st_point[id].y;
                    // if (check_br(x, y))
                    if (check_br(x, y) || vec_init_mask[1](y, x) == 0)
                        vec_sep_mask[id](r, c) = 0;
                }
            }
        }

        void IPM_SeperateMap(vector<MatrixI> &vec_sep_map, vector<MatrixUD> &vec_sep_mask, vector<Pt2i> &vec_st_point,
                             vector<MatrixI> &vec_round_map, vector<MatrixUD> &tmp_sep_mask,
                             vector<Pt2i> &tmp_st_point) {
            int map_num = 12;
            vec_sep_map.resize(map_num);
            vec_sep_mask.resize(map_num);
            vec_st_point.resize(map_num);

            vector<int> vec_camera_index = {3, 0, 1, 3, 1, 3, 2, 1, 0, 0, 2, 2};
            vector<int> vec_map_index = {0, 1, 2, 3, 4, 5, 6, 7, 0, 2, 5, 7};
            for (int i = 0; i < map_num; i++) {
                int map_id = vec_map_index[i];
                int camera_id = vec_camera_index[i];

                auto &  pt = tmp_st_point[map_id];
                Size size = tmp_sep_mask[map_id].size();
                vec_sep_map[i].set(size.height, size.width, 2);
                // MLOG_INFO("init map %d %d, sep size %d %d, st %d %d",
                // vec_round_map[camera_id].rows(), vec_round_map[camera_id].cols(),
                // vec_sep_map[i].rows(), vec_sep_map[i].cols(), pt.y, pt.x);

                for (int r = 0; r < size.height; r++) {
                    for (int c = 0; c < size.width; c++) {
                        vec_sep_map[i](r, c, 0) = vec_round_map[camera_id](r + pt.y, c + pt.x, 0);
                        vec_sep_map[i](r, c, 1) = vec_round_map[camera_id](r + pt.y, c + pt.x, 1);
                    }
                }

                vec_sep_mask[i] = tmp_sep_mask[map_id];
                vec_st_point[i] = tmp_st_point[map_id];
            }
        }

        Status IPM_RoundMap(MatrixI& round_map, MatrixF& src_map) {
            round_map.set(src_map.rows(), src_map.cols(), 2);
            for (int r = 0; r < src_map.rows(); r++) {
                for (int c = 0; c < src_map.cols(); c++) {
                    round_map(r, c, 0) = (int)round(src_map(r, c, 0));
                    round_map(r, c, 1) = (int)round(src_map(r, c, 1));
                }
            }

            return Status::OK;
        }

        Status IPM_RoundMap(vector<MatrixI> &vec_round_map, vector<MatrixF> &vec_sep_map) {
            int map_num = vec_sep_map.size();
            if (map_num <= 0)
                return Status::INVALID_ARGUMENT;

            vec_round_map.resize(map_num);
            for (int map_id = 0; map_id < map_num; map_id++) {
                MatrixF &map_f = vec_sep_map[map_id];
                vec_round_map[map_id].set(map_f.rows(), map_f.cols(), 2);
                for (int r = 0; r < map_f.rows(); r++) {
                    for (int c = 0; c < map_f.cols(); c++) {
                        vec_round_map[map_id](r, c, 0) = (int)round(vec_sep_map[map_id](r, c, 0));
                        vec_round_map[map_id](r, c, 1) = (int)round(vec_sep_map[map_id](r, c, 1));
                    }
                }
            }

            return Status::OK;
        }

        Status IPM_RoundMap2NV12(vector<MatrixI> &vec_oper_map, vector<MatrixI> &vec_round_map,
                                 vector<MatrixUD> &vec_sep_mask, vector<Pt2i> &vec_st_point, Size &image_size,
                                 Size &stitch_size) {
            int map_num = vec_round_map.size();
            int src_width = image_size.width;

            int dst_width = stitch_size.width;
            vec_oper_map.resize(map_num);
            Pt2i tl_point(vec_st_point[0].x, vec_st_point[1].y);
            for (int i = 0; i < map_num; i++) {
                Pt2i tl = vec_st_point[i] - tl_point;
                Size map_size = vec_round_map[i].size();
                vec_oper_map[i].set((int)round(map_size.height * 1.5), map_size.width, 1, 0);
                int uv_row = map_size.height;
                for (int r = 0; r < map_size.height; r++) {
                    const uchar *ptr_mask = vec_sep_mask[i].ptr(r);
                    const int *  ptr_map = vec_round_map[i].ptr(r);
                    auto &       oper_map = vec_oper_map[i];
                    for (int c = 0; c < map_size.width; c++) {
                        if (ptr_mask[c] == 255) {
                            int mx = ptr_map[c * 2];
                            int my = ptr_map[c * 2 + 1];
                            int x = my * src_width + mx;
                            oper_map(r, c) = x;
                            if (r % 2 == 0) {
                                int src_uv_idx = int(my / 2 + image_size.height) * src_width + mx;
                                int y;
                                int dst_uv_idx = ((r + tl.y) / 2 + stitch_size.height) * dst_width + c + tl.x;
                                if (dst_uv_idx % 2 == 0) {
                                    if (src_uv_idx % 2 == 0)
                                        y = src_uv_idx;
                                    else
                                        y = src_uv_idx + 1;
                                } else {
                                    if (src_uv_idx % 2 == 0)
                                        y = src_uv_idx + 1;
                                    else
                                        y = src_uv_idx;
                                }
                                oper_map(uv_row, c) = y;
                            }
                        }
                    }
                    if (r % 2 == 1)
                        uv_row++;
                }
            }

            return Status::OK;
        }

        Status IPM_RoundMap2NV16(vector<MatrixI> &vec_oper_map, vector<MatrixI> &vec_round_map,
                                 vector<MatrixUD> &vec_sep_mask, vector<Pt2i> &vec_st_point, Size &image_size,
                                 Size &stitch_size) {
            int map_num = vec_round_map.size();
            int src_width = image_size.width;
            int src_shift = image_size.width * image_size.height;
            int dst_width = stitch_size.width;
            vec_oper_map.resize(map_num);
            Pt2i tl_point(vec_st_point[0].x, vec_st_point[1].y);
            for (int i = 0; i < map_num; i++) {
                Pt2i tl = vec_st_point[i] - tl_point;
                Size map_size = vec_round_map[i].size();
                vec_oper_map[i].set(map_size.height, map_size.width, 2, 0);
                for (int r = 0; r < map_size.height; r++) {
                    const uchar *ptr_mask = vec_sep_mask[i].ptr(r);
                    const int *  ptr_map = vec_round_map[i].ptr(r);
                    auto &       oper_map = vec_oper_map[i];
                    for (int c = 0; c < map_size.width; c++) {
                        if (ptr_mask[c] == 255) {
                            int mx = ptr_map[c * 2];
                            int my = ptr_map[c * 2 + 1];
                            int x = my * src_width + mx;
                            int y;
                            if (((r + tl.y) * dst_width + c + tl.x) % 2 == 0) {
                                if (x % 2 == 0)
                                    y = src_shift + x;
                                else
                                    y = src_shift + x + 1;
                            } else {
                                if (x % 2 == 0)
                                    y = src_shift + x + 1;
                                else
                                    y = src_shift + x;
                            }
                            oper_map(r, c, 0) = x;
                            oper_map(r, c, 1) = y;
                        }
                    }
                }
            }

            return Status::OK;
        }

        Status IPM_RoundMap2YUYV(vector<MatrixI> &vec_oper_map, vector<MatrixI> &vec_round_map,
                                 vector<MatrixUD> &vec_sep_mask, vector<Pt2i> &vec_st_point, Size &image_size,
                                 Size &stitch_size) {
            int map_num = vec_round_map.size();
            vec_oper_map.resize(map_num);
            int     src_width = image_size.width;
            int     dst_width = stitch_size.width;
            Pt2i tl_point(vec_st_point[0].x, vec_st_point[1].y);
            for (int i = 0; i < map_num; i++) {
                Pt2i tl = vec_st_point[i] - tl_point;
                Size map_size = vec_round_map[i].size();
                vec_oper_map[i].set(map_size.height, map_size.width, 2, 0);
                for (int r = 0; r < map_size.height; r++) {
                    const uchar *ptr_mask = vec_sep_mask[i].ptr(r);
                    const int *  ptr_map = vec_round_map[i].ptr(r);
                    auto &       oper_map = vec_oper_map[i];
                    for (int c = 0; c < map_size.width; c++) {
                        if (ptr_mask[c] == 255) {
                            int mx = ptr_map[c * 2];
                            int my = ptr_map[c * 2 + 1];
                            int x = 2 * (my * src_width + mx);
                            int y;
                            if (((r + tl.y) * dst_width + c + tl.x) % 2 == 0) // for dst u
                            {
                                if (x % 4 == 0)
                                    y = x;
                                else
                                    y = x - 2;
                            } else // for dst v
                            {
                                if (x % 4 == 0)
                                    y = x + 2;
                                else
                                    y = x;
                            }
                            oper_map(r, c, 0) = y;
                            oper_map(r, c, 1) = x+1;
                        }
                    }
                }
            }

            return Status::OK;
        }

        Status IPM_get_blank_mask(BlankArea& blank_area, uint8_t** blank_mask)
        {
            if (!g_init) {
                return Status::FAILURE;
            }

            blank_area = g_lut_imp.blank_area;
            *blank_mask = g_lut_imp.blank_mask.data();
            return Status::OK;
        }

        void IPM_CheckMapAndMask(MatrixI& round_map, MatrixUD& mask, Size& image_size)
        {
            if(round_map.rows()!=mask.rows() || round_map.cols()!=mask.cols())
            {
                return;
            }

            for (int r = 0; r < round_map.rows(); r++) {
                for (int c = 0; c < round_map.cols(); c++) {
                    int x = round_map(r, c, 0);
                    int y = round_map(r, c, 1);
                    if(x<0 || x>=image_size.width || y<0 || y>=image_size.height)
                    {
                        round_map(r, c, 0) = 0;
                        round_map(r, c, 1) = 0;
                        mask(r, c) = 0;
                    }
                }
            }
        }
    }
} // namespace haomo

