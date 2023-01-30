#ifndef STITCH_LUT_INFO_H_
#define STITCH_LUT_INFO_H_

#include "matrix.hpp"
#include "stitch_interface.h"
#include <vector>

namespace haomo {
namespace stitch {
struct LutInfoImp {
    std::vector<MatrixI>  vec_oper_map;
    std::vector<MatrixUD> vec_sep_mask;
    std::vector<Pt2i>     vec_st_point;
    Size                  image_size, stitch_size;
    int                      data_type;
    BlankArea                blank_area;
    MatrixUD              blank_mask;
};

void printLutInfo(LutInfoImp &lut_info);

Status lutImp2LutInfo(LutInfoImp &info_imp, LutInfo &lut_info);

Status lutInfo2LutImp(LutInfo &lut_info, LutInfoImp &info_imp);

extern LutInfoImp g_lut_imp;
extern bool       g_init;
}
} // namespace haomo

#endif
