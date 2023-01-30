#include "lut_info.h"

using namespace std;
namespace haomo {
namespace stitch {
void printLutInfo(LutInfoImp &lut_info) {
    MLOG_INFO("lut_info.data_type: %d\n", lut_info.data_type);
    MLOG_INFO("lut_info.image_size.width: %d\n", lut_info.image_size.width);
    MLOG_INFO("lut_info.image_size.height: %d\n", lut_info.image_size.height);
    MLOG_INFO("lut_info.output_size.width: %d\n", lut_info.stitch_size.width);
    MLOG_INFO("lut_info.output_size.height: %d\n", lut_info.stitch_size.height);

    for (uint32_t i = 0; i < lut_info.vec_st_point.size(); i++) {
        MLOG_INFO("points index:%u, x:%d, y:%d\n", i, lut_info.vec_st_point[i].x, lut_info.vec_st_point[i].y);
    }

    for (uint32_t i = 0; i < lut_info.vec_oper_map.size(); i++) {
        MLOG_INFO("map index:%u, rows:%d, cols:%d, channels:%d\n", i, lut_info.vec_oper_map[i].rows(),
               lut_info.vec_oper_map[i].cols(), lut_info.vec_oper_map[i].channels());
        // uint32_t data_len = lut_info.vec_oper_map[i].rows() *
        // lut_info.vec_oper_map[i].cols() * lut_info.vec_oper_map[i].channels();
        // for (uint32_t j = 0; j < data_len; j++)
        // {
        //     if (j < 10)
        //     {
        //         int *map_data = lut_info.vec_oper_map[i].data();
        //         printf("data[%u]:%d\n", j, map_data[j]);
        //     }
        // }
    }

    for (uint32_t i = 0; i < lut_info.vec_sep_mask.size(); i++) {
        MLOG_INFO("map index:%u, rows:%d, cols:%d, channels:%d\n", i, lut_info.vec_sep_mask[i].rows(),
               lut_info.vec_sep_mask[i].cols(), lut_info.vec_sep_mask[i].channels());
    }
}

static uint32_t CheckCrc32WithInitial(uint8_t *ucBuf, uint32_t uiLen, uint32_t uiSeed) {
    uint32_t crc = uiSeed; /*0x00000000;*/
    if (NULL == ucBuf || uiLen <= 0) {
        return crc;
    }

    uint8_t *ucOld = ucBuf;
    uint32_t rem = uiLen % sizeof(unsigned int);
    if (rem > 0) {
        int n = sizeof(unsigned int) - rem;
        for (int i = 0; i < n; i++) {
            ucOld[uiLen + i] = 0xff;
        }
        uiLen += n;
    }

    uint32_t uiCount = uiLen / sizeof(unsigned int);
    for (uint32_t i = 0; i < uiCount; i++) {
        uint32_t uiTemp = *(uint32_t *)ucOld;
        for (uint32_t j = 0; j < 32; j++) {
            if ((crc ^ uiTemp) & 0x80000000) {
                crc = 0x04C11DB7 ^ (crc << 1);
            } else {
                crc <<= 1;
            }
            uiTemp <<= 1;
        }
        ucOld += sizeof(unsigned int);
    }

    return crc;
}

Status lutImp2LutInfo(LutInfoImp &lut_imp, LutInfo &lut_info) {
    MLOG_INFO("!!!output lut info!!!\n");
    uint32_t lut_len =
        4 + sizeof(Size) * 2 + sizeof(uint32_t) * 3 + sizeof(Pt2i) * lut_imp.vec_st_point.size() + 4 + 4;

    for (uint32_t i = 0; i < lut_imp.vec_oper_map.size(); i++) {
        lut_len += sizeof(int) * 3; // rows, cols, channels
        lut_len += lut_imp.vec_oper_map[i].rows() * lut_imp.vec_oper_map[i].cols() *
                   lut_imp.vec_oper_map[i].channels() * sizeof(int);
    }

    for (uint32_t i = 0; i < lut_imp.vec_sep_mask.size(); i++) {
        lut_len += sizeof(int) * 3; // rows, cols, channels
        lut_len += lut_imp.vec_sep_mask[i].rows() * lut_imp.vec_sep_mask[i].cols() *
                   lut_imp.vec_sep_mask[i].channels() * sizeof(uchar);
    }
    // lut_len = len + image + output + point_size + map_size + mask_size + crc +
    // point_size*point_len + map_size*map_len + mask_size*mask_len

    uint8_t *g_lut_data = new uchar[lut_len];
    // LCOV_EXCL_START
    if (g_lut_data == nullptr) {
        MLOG_ERROR("get g_lut_data memrory failure\n");
        return Status::FAILURE;
    }
    // LCOV_EXCL_STOP
    lut_info.lut_buffer = g_lut_data;
    lut_info.lut_buffer_size = lut_len;

    int *lut_header = (int *)g_lut_data;
    *lut_header++ = (int)lut_len;
    *lut_header++ = lut_imp.image_size.width;
    *lut_header++ = lut_imp.image_size.height;
    *lut_header++ = lut_imp.stitch_size.width;
    *lut_header++ = lut_imp.stitch_size.height;
    *lut_header++ = lut_imp.vec_st_point.size();
    *lut_header++ = lut_imp.vec_oper_map.size();
    *lut_header++ = lut_imp.vec_sep_mask.size();
    *lut_header++ = lut_imp.data_type;

    int *point_data = lut_header;
    for (uint32_t i = 0; i < lut_imp.vec_st_point.size(); i++) {
        *point_data++ = lut_imp.vec_st_point[i].x;
        *point_data++ = lut_imp.vec_st_point[i].y;
    }

    int *map_data = point_data;
    for (uint32_t i = 0; i < lut_imp.vec_oper_map.size(); i++) {
        *map_data++ = lut_imp.vec_oper_map[i].rows();
        *map_data++ = lut_imp.vec_oper_map[i].cols();
        *map_data++ = lut_imp.vec_oper_map[i].channels();
        uint32_t data_len =
            lut_imp.vec_oper_map[i].rows() * lut_imp.vec_oper_map[i].cols() * lut_imp.vec_oper_map[i].channels();
        memcpy((uint8_t *)map_data, (uint8_t *)lut_imp.vec_oper_map[i].data(), data_len * sizeof(int));

        map_data += data_len;
    }

    uchar *mask_data = (uchar *)map_data;
    for (uint32_t i = 0; i < lut_imp.vec_sep_mask.size(); i++) {
        *mask_data++ = lut_imp.vec_sep_mask[i].rows() & 0xFF;
        *mask_data++ = (lut_imp.vec_sep_mask[i].rows() & 0xFF00) >> 8;
        *mask_data++ = (lut_imp.vec_sep_mask[i].rows() & 0xFF0000) >> 16;
        *mask_data++ = (lut_imp.vec_sep_mask[i].rows() & 0xFF000000) >> 24;

        *mask_data++ = lut_imp.vec_sep_mask[i].cols() & 0xFF;
        *mask_data++ = (lut_imp.vec_sep_mask[i].cols() & 0xFF00) >> 8;
        *mask_data++ = (lut_imp.vec_sep_mask[i].cols() & 0xFF0000) >> 16;
        *mask_data++ = (lut_imp.vec_sep_mask[i].cols() & 0xFF000000) >> 24;

        *mask_data++ = lut_imp.vec_sep_mask[i].channels() & 0xFF;
        *mask_data++ = (lut_imp.vec_sep_mask[i].channels() & 0xFF00) >> 8;
        *mask_data++ = (lut_imp.vec_sep_mask[i].channels() & 0xFF0000) >> 16;
        *mask_data++ = (lut_imp.vec_sep_mask[i].channels() & 0xFF000000) >> 24;

        uint32_t data_len =
            lut_imp.vec_sep_mask[i].rows() * lut_imp.vec_sep_mask[i].cols() * lut_imp.vec_sep_mask[i].channels();
        memcpy((uint8_t *)mask_data, (uint8_t *)lut_imp.vec_sep_mask[i].data(), data_len * sizeof(uchar));
        mask_data += data_len;
    }

    // printf_lut_info();

    uint8_t *crc_data = (uint8_t *)mask_data;
    uint32_t crc = CheckCrc32WithInitial(lut_info.lut_buffer, lut_info.lut_buffer_size - 4, 0);
    *crc_data++ = crc & 0xFF;
    *crc_data++ = (crc & 0xFF00) >> 8;
    *crc_data++ = (crc & 0xFF0000) >> 16;
    *crc_data++ = (crc & 0xFF000000) >> 24;
    return Status::OK;
}

Status lutInfo2LutImp(LutInfo &lut_info, LutInfoImp &lut_imp) {
    MLOG_INFO("!!!get lut info from files!!!\n");
    uint32_t lut_len = (*lut_info.lut_buffer) | (*(lut_info.lut_buffer + 1)) << 8 | (*(lut_info.lut_buffer + 2)) << 16 |
                       (*(lut_info.lut_buffer + 3)) << 24;
    if (lut_len > lut_info.lut_buffer_size)
        return Status::FAILURE;
    uint8_t *crc_data = lut_info.lut_buffer + lut_info.lut_buffer_size - 4;
    uint32_t crc_bin = (*crc_data) | (*(crc_data + 1)) << 8 | (*(crc_data + 2)) << 16 | (*(crc_data + 3)) << 24;
    uint32_t crc = CheckCrc32WithInitial(lut_info.lut_buffer, lut_info.lut_buffer_size - 4, 0);
    MLOG_INFO("crc bin %u, crc %u, buffer_size %d\n", crc_bin, crc, lut_info.lut_buffer_size);
    if (crc_bin != crc)
        return Status::FAILURE;
    // SLOG_ASSERT(crc_bin == crc);
    // uint8_t* g_lut_data = new uint8_t[lut_info.lut_buffer_size];
    // if (g_lut_data == nullptr)
    //{
    //    SLOG_ERROR("get g_lut_data memrory failure\n");
    //    return Status::_FAILURE;
    //}
    // memcpy(g_lut_data, lut_info.lut_buffer, lut_info.lut_buffer_size);

    uint8_t *g_lut_data = lut_info.lut_buffer;
    int *    lut_header = (int *)(g_lut_data + 4);
    lut_imp.image_size.width = *lut_header++;
    lut_imp.image_size.height = *lut_header++;
    lut_imp.stitch_size.width = *lut_header++;
    lut_imp.stitch_size.height = *lut_header++;
    uint32_t point_size = *lut_header++;
    uint32_t map_size = *lut_header++;
    uint32_t mask_size = *lut_header++;
    lut_imp.data_type = *lut_header++;

    int *point_data = lut_header;
    lut_imp.vec_st_point.resize(point_size);
    for (uint32_t i = 0; i < point_size; i++) {
        lut_imp.vec_st_point[i].x = *point_data++;
        lut_imp.vec_st_point[i].y = *point_data++;
    }

    int  rows, cols, channel;
    int *map_data = point_data;
    lut_imp.vec_oper_map.resize(map_size);
    for (uint32_t i = 0; i < lut_imp.vec_oper_map.size(); i++) {
        rows = *map_data++;
        cols = *map_data++;
        channel = *map_data++;

        lut_imp.vec_oper_map[i] = Matrix<int>(rows, cols, channel, map_data);
        map_data += rows * cols * channel;
    }

    uchar *mask_data = (uchar *)map_data;
    lut_imp.vec_sep_mask.resize(mask_size);

    for (uint32_t i = 0; i < lut_imp.vec_sep_mask.size(); i++) {
        rows = (*mask_data) | (*(mask_data + 1)) << 8 | (*(mask_data + 2)) << 16 | (*(mask_data + 3)) << 24;
        mask_data += 4;

        cols = (*mask_data) | (*(mask_data + 1)) << 8 | (*(mask_data + 2)) << 16 | (*(mask_data + 3)) << 24;
        mask_data += 4;

        channel = (*mask_data) | (*(mask_data + 1)) << 8 | (*(mask_data + 2)) << 16 | (*(mask_data + 3)) << 24;
        mask_data += 4;

        lut_imp.vec_sep_mask[i] = Matrix<uchar>(rows, cols, channel, mask_data);
        mask_data += rows * cols * channel;
    }

    return Status::OK;
}

Status IPM_release_lut(LutInfo *lut_info) {
    if (lut_info == nullptr)
        return Status::FAILURE;
    lut_info->lut_buffer_size = 0;
    int flag = 0;
    if (g_init)
        g_init = false;
    else
        flag |= 1;
    if (lut_info->lut_buffer != nullptr) {
        delete[] lut_info->lut_buffer;
        lut_info->lut_buffer = nullptr;
    } else
        flag |= 1;
    return flag == 0 ? Status::OK : Status::FAILURE;
}
}
} // namespace haomo
