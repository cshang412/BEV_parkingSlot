#ifndef STITCH_Matrix_HPP_
#define STITCH_Matrix_HPP_

#include "mini_log.hpp"
#include <exception>
#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

namespace haomo {
namespace stitch{
template <typename T> struct Pt2 {
    Pt2(T x_ = T(0), T y_ = T(0)) : x(x_), y(y_) {
    }
    Pt2 operator-(Pt2 &pt) {
        return Pt2(x - pt.x, y - pt.y);
    }
    T x, y;
};

typedef Pt2<int>    Pt2i;
typedef Pt2<float>  Pt2f;
typedef Pt2<double> Pt2d;

template <typename T> struct Pt3 {
    Pt3(T x_ = T(0), T y_ = T(0), T z_ = T(0)) : x(x_), y(y_), z(z_) {
    }
    T x, y, z;
};

typedef Pt3<int>    Pt3i;
typedef Pt3<float>  Pt3f;
typedef Pt3<double> Pt3d;

struct Size {
    Size(int w = 0, int h = 0) : width(w), height(h) {
    }
    int width, height;

    bool operator!=(const Size &A) {
        return width != A.width || height != A.height;
    }
};

struct Rect {
    Rect(int x_ = 0, int y_ = 0, int w = 0, int h = 0) : x(x_), y(y_), width(w), height(h) {
    }
    int     x, y, width, height;
    Pt2i tl() {
        return Pt2i(x, y);
    }
    Pt2i br() {
        return Pt2i(x + width - 1, y + height - 1);
    }
};

template <typename T> class Matrix {
  public:
    Matrix();
    Matrix(const Matrix<T> &A);
    Matrix(int rows, int cols, int channel, const T *data);
    Matrix(int rows, int cols, int channel = 1, const T &c = T(0));
    ~Matrix();

    Matrix<T> &operator=(const Matrix<T> &A);
    Matrix<T> &operator=(const T &c);
    Matrix<T> &operator*(const T &c);
    Matrix<T> &operator+(const Matrix<T> &A);

    int channels() const {
        return channels_;
    };
    int rows() const {
        return rows_;
    };
    int cols() const {
        return cols_;
    };
    Size size() const {
        return Size(cols_, rows_);
    };
    int area() const {
        return rows_ * cols_ * channels_;
    };
    void setIdentity();
    void transpose(Matrix &new_mat);
    void release();

    T &      operator()(int r, int c, int ch = 0);
    T        operator()(int r, int c, int ch = 0) const;
    const T *ptr(int r);
    T *      data() const;

    void set(int rows, int cols, int channels = 1, T c = T(0));

    // Matrix<T> operator*(Matrix<T>& mat);

  private:
    int rows_ = 0;
    int cols_ = 0;
    int channels_ = 0;
    T * data_ = nullptr;

    void init(int rows, int cols, int channels = 1);
    void copy(const Matrix<T> &M);
};

typedef Matrix<double>        MatrixD;
typedef Matrix<float>         MatrixF;
typedef Matrix<unsigned char> MatrixUD;
typedef Matrix<int>           MatrixI;
typedef unsigned char            uchar;

template <typename T> void Matrix<T>::setIdentity() {
    for (int r = 0; r < rows_ && r < cols_; r++) {
        for (int ch = 0; ch < channels_; ch++)
            data_[(r * cols_ + r) * channels_ + ch] = 1;
    }
}

template <typename T> void Matrix<T>::init(int rows, int cols, int channels) {
    rows_ = rows;
    cols_ = cols;
    channels_ = channels;
    if (area() <= 0) {
        rows_ = cols_ = channels_ = 0;
        return;
    }
    data_ = nullptr;
    data_ = new T[rows * cols * channels_];

    // LCOV_EXCL_START
    if (data_ == nullptr) {
        rows_ = cols_ = channels_ = 0;
        MLOG_ERROR("new matrix failed!, just exit");
        exit(-1);
    }
    // LCOV_EXCL_STOP

    int Matrix_size = area();
    memset((uint8_t *)data_, 0, sizeof(T) * Matrix_size);
}

template <typename T> void Matrix<T>::set(int rows, int cols, int channels, T con) {
    init(rows, cols, channels);
    int Matrix_size = area();
    if (Matrix_size <= 0)
        return;
    for (int r = 0; r < rows_; r++) {
        for (int c = 0; c < cols_; c++) {
            for (int ch = 0; ch < channels_; ch++)
                data_[(r * cols_ + c) * channels_ + ch] = con;
        }
    }
}

template <typename T> void Matrix<T>::copy(const Matrix<T> &M) {
    init(M.rows(), M.cols(0, M.channels()));
}

template <typename T> Matrix<T>::Matrix() : rows_(0), cols_(0), channels_(0), data_(nullptr) {
}

template <typename T> Matrix<T>::Matrix(const Matrix<T> &A) {
    int rows = A.rows();
    int cols = A.cols();
    int channels = A.channels();
    init(rows, cols, channels);
    int Matrix_size = area();
    if (Matrix_size <= 0)
        return;
    int element_size = sizeof(T);
    memcpy((uint8_t *)data_, (uint8_t *)A.data(), Matrix_size * element_size);
}

// LCOV_EXCL_START
template <typename T> void Matrix<T>::release() {
    rows_ = cols_ = channels_ = 0;
    if (data_ != nullptr) {
        delete[] data_;
        data_ = nullptr;
    }
}
// LCOV_EXCL_STOP

template <typename T> Matrix<T>::~Matrix() {
    release();
}

template <typename T> Matrix<T>::Matrix(int rows, int cols, int channel, const T *data) {
    init(rows, cols, channel);
    int Matrix_size = area();
    if (Matrix_size <= 0)
        return;
    memcpy((uint8_t *)data_, (uint8_t *)data, sizeof(T) * Matrix_size);
}

template <typename T> Matrix<T>::Matrix(int rows, int cols, int channel, const T &con) {
    init(rows, cols, channel);
    int Matrix_size = area();
    if (Matrix_size <= 0)
        return;
    for (int r = 0; r < rows_; r++) {
        for (int c = 0; c < cols_; c++) {
            for (int ch = 0; ch < channels_; ch++)
                data_[(r * cols_ + c) * channels_ + ch] = con;
        }
    }
}

template <typename T> Matrix<T> &Matrix<T>::operator=(const Matrix<T> &A) {
    if (this == &A)
        return *this;

    init(A.rows(), A.cols(), A.channels());
    int Matrix_size = area();
    if (Matrix_size <= 0)
        return *this;
    memcpy((uint8_t *)this->data_, (uint8_t *)A.data(), sizeof(T) * Matrix_size);
    return *this;
}

template <typename T> Matrix<T> &Matrix<T>::operator*(const T &cc) {
    for (int r = 0; r < rows_; r++) {
        for (int c = 0; c < cols_; c++) {
            for (int ch = 0; ch < channels_; ch++)
                data_[(r * cols_ + c) * channels_ + ch] *= cc;
        }
    }
    return *this;
}

template <typename T> Matrix<T> &Matrix<T>::operator=(const T &cc) {
    int Matrix_size = area();
    if (Matrix_size <= 0)
        return *this;
    for (int r = 0; r < rows_; r++) {
        for (int c = 0; c < cols_; c++) {
            for (int ch = 0; ch < channels_; ch++)
                data_[(r * cols_ + c) * channels_ + ch] = cc;
        }
    }
    return *this;
}

// LCOV_EXCL_START
template <typename T> T &Matrix<T>::operator()(int r, int c, int ch) {
    MLOG_ASSERT(r >= 0 && c >= 0 && ch >= 0);
    MLOG_ASSERT(r < rows_ && c < cols_ && ch < channels_);
    int idx = (r * cols_ + c) * channels_ + ch;
    return data_[idx];
}

template <typename T> T Matrix<T>::operator()(int r, int c, int ch) const {
    MLOG_ASSERT(r >= 0 && c >= 0 && ch >= 0);
    MLOG_ASSERT(r < rows_ && c < cols_ && ch < channels_);
    int idx = (r * cols_ + c) * channels_ + ch;
    return data_[idx];
}

template <typename T> const T *Matrix<T>::ptr(int r) {
    MLOG_ASSERT(rows_ > 0 && cols_ > 0 && channels_ > 0 && r < rows_);
    int idx = r * cols_ * channels_;
    return data_ + idx;
}
// LCOV_EXCL_STOP

template <typename T> T *Matrix<T>::data() const {
    return data_;
}

template <typename T> void Matrix<T>::transpose(Matrix &new_mat) {
    int Matrix_size = area();
    if (Matrix_size <= 0)
        return;

    new_mat.set(cols_, rows_, channels_, 0);
    for (int r = 0; r < rows_; r++) {
        for (int c = 0; c < cols_; c++) {
            for (int ch = 0; ch < channels_; ch++)
                new_mat(c, r, ch) = data_[(r * cols_ + c) * channels_ + ch];
        }
    }
}

template <typename T> Matrix<T> &Matrix<T>::operator+(const Matrix<T> &A) {
    if (rows_ != A.rows() || cols_ != A.cols() || channels_ != A.channels()) {
        MLOG_ERROR("when add, matrix rows cols channels must be the same!");
        return *this;
    }

    for (int r = 0; r < rows_; r++) {
        for (int c = 0; c < cols_; c++) {
            for (int ch = 0; ch < channels_; ch++)
                data_[(r * cols_ + c) * channels_ + ch] += A(r, c, ch);
        }
    }

    return *this;
}
}
} // namespace haomo

#endif
