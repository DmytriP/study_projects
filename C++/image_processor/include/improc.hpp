#ifndef IMAGE_PROCESSOR_IMPROC_HPP
#define IMAGE_PROCESSOR_IMPROC_HPP

#include "bitmap.h"
#include <vector>
#include <iostream>
#include <string>
#include <stdio.h>
#include <cstring>
#include <functional>

#define BLACK 0

template<typename T>
class Matrix {

    size_t row_num_;
    size_t col_num_;
    std::vector<std::vector<T>> matrix;

public:
    Matrix(size_t row_num, size_t col_num, T value) : row_num_(row_num), col_num_(col_num) {
        for (int i = 0; i < row_num; i++) {
            std::vector<T> v(col_num);
            for (int j = 0; j < col_num; j++)
                v[j] = value;
            matrix.push_back(v);
        }
    }

    std::vector<T>& operator[](size_t idx) { return matrix[idx]; }

    auto begin() { return matrix.begin(); }

    auto size() { return matrix.size(); }

    auto end() { return matrix.end(); }

    size_t rows() { return row_num_; }

    size_t colums() { return col_num_; }

};

using Mask = Matrix<double>;


class Images : public Matrix<byte> {

    BITMAPINFO* Bitmapinfo;
    BITMAPFILEHEADER bmp_;

public:
    Images(size_t h, size_t w, BITMAPFILEHEADER bmp, BITMAPINFO* bitmap_ptr) : Matrix(h, w, 0), bmp_(bmp) {

        size_t bitmap_info_size = bmp.bfOffBits - sizeof(BITMAPFILEHEADER);
        Bitmapinfo = (BITMAPINFO*) malloc(bitmap_info_size);
        std::memcpy(Bitmapinfo, bitmap_ptr, bitmap_info_size);
    }

    BITMAPFILEHEADER get_bmp(){return bmp_;}
    BITMAPINFO* get_Bitmapinfo(){return Bitmapinfo;}


    ~Images() { free(Bitmapinfo); }
    Images(const Images&) = default;

};

extern Images load_bitmap(const std::string& filename);

extern int save_bitmap(const std::string& filename, Images &image_array);

extern  Mask make_mask(size_t mask_size);

extern void inverse(int i, int j, Images &image);

extern Images& transform(Images& image, std::function<void (int, int, Images&)> function);

extern void filtrate(int i, int j, Images& img);


#endif //IMAGE_PROCESSOR_IMPROC_HPP
