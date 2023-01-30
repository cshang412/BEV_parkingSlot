#ifndef STITCH_MINI_LOG_HPP_
#define STITCH_MINI_LOG_HPP_

#include <stdio.h>

#define COLOR_NONE "\033[0m"
#define COLOR_RED "\033[0;32;31m"
#define COLOR_CYAN "\033[0;36m"
#define COLOR_LIGHT_YELLOW "\033[1;33m"
#define COLOR_PURPLE "\033[0;35m"
#define COLOR_GREEN "\033[0;32;32m"
#define CLEAR_COLOR printf(COLOR_NONE)

#define PKL_PARSER(S) S

#define MLOG_INFO(...)                                                                                                 \
    {                                                                                                                  \
        printf(COLOR_GREEN "[INFO]: " __VA_ARGS__);                                                                    \
        printf("\n");                                                                                                  \
        CLEAR_COLOR;                                                                                                   \
    }

#define MLOG_IINFO(...)                                                                                                \
    {                                                                                                                  \
        printf(COLOR_PURPLE "[IINFO]: " __VA_ARGS__);                                                                  \
        printf("\n");                                                                                                  \
        CLEAR_COLOR;                                                                                                   \
    }

#define MLOG_WARN(...)                                                                                                 \
    {                                                                                                                  \
        printf(COLOR_LIGHT_YELLOW "[WARN]: " __VA_ARGS__);                                                             \
        printf("\n");                                                                                                  \
        CLEAR_COLOR;                                                                                                   \
    }

#define MLOG_ERROR(...)                                                                                                \
    {                                                                                                                  \
        printf(COLOR_RED "[ERROR]: " __VA_ARGS__);                                                                     \
        printf("\n");                                                                                                  \
        CLEAR_COLOR;                                                                                                   \
    }

#define MLOG_FATAL(...)                                                                                                \
    {                                                                                                                  \
        printf(COLOR_RED "[FATAL]: " __VA_ARGS__);                                                                     \
        printf("\n");                                                                                                  \
        CLEAR_COLOR;                                                                                                   \
    }

#define MLOG_ASSERT(condition)                                                                                         \
    {                                                                                                                  \
        if (!(condition)) {                                                                                            \
            MLOG_FATAL("%s", #condition);                                                                              \
            exit(-1);                                                                                                  \
        }                                                                                                              \
    }

#endif
