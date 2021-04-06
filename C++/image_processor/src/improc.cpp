#include "improc.hpp"
#include "bitmap.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <improc.hpp>


Images load_bitmap(const std::string& filename)
{
    BITMAPINFO *BitmapInfo;
    BITMAPFILEHEADER header;
    byte* bitmapBytes = LoadDIBitmap(filename.c_str(), &BitmapInfo, &header);

    if (!bitmapBytes) {
        std::string FileIOError  = " Błąd podczas odczytu danych z pliku.";
        throw FileIOError ;
    }

    auto h = (size_t) (BitmapInfo)->bmiHeader.biHeight;
    auto w = (size_t) (BitmapInfo)->bmiHeader.biWidth;
    int bits_per_pixel = (BitmapInfo)->bmiHeader.biBitCount;

    /* see: https://en.wikipedia.org/wiki/BMP_file_format#Pixel_storage */
    size_t row_size = (bits_per_pixel * w + 31) / 32 * 4;

    printf("Successfully loaded a %zux%zu image - %s.\n\n", h, w, filename.c_str());

    Images image_array(h, w, header, BitmapInfo);

    byte* reader = bitmapBytes;

    // The order of the pixels in BMP file is as follows: from left to right, from bottom to top (first pixel is from
    // lower left corner of the picture).
    for (size_t i = 0; i < h; ++i) {
        for (size_t j = 0; j < w; ++j) {
            image_array[h - i - 1][j] = *reader++;
        }

        reader += row_size - w;
    }

    free(bitmapBytes);
    free(BitmapInfo);
    return image_array;
}



Mask make_mask(size_t mask_size) {
    double value = 1./(double)(mask_size*mask_size);
    return  Mask(mask_size, mask_size, value);
}



Images& transform(Images& image, std::function<void(int, int, Images&)> function){
    for (int i = 0; i <image.size() ; ++i) {
        for (int j = 0; j < image.size(); ++j) {
            function(i,j,image);
        }
    }
    return image;
}

void inverse(int i, int j, Images &image){
    image[i][j]= (255-image[i][j]);
}

void filtrate(int i, int j, Images& img)
{
    int size_mask=7;
    int size_0= (size_mask)/2;
    int size_1 =(size_mask)/2;


    if ((i - size_0 < 0) || (i+size_1 >= img.size()) || (j-size_0 < 0 ) || (j + size_1 >= img.size()))
    {
        img[i][j] = BLACK;
        return;
    }
    Mask m = make_mask(size_mask);
    double new_value = 0;
    for(int a= -size_0; a <= size_0; a++ )
    {
        for(int b=-size_1; b <= size_1; b++)
        {
            new_value += m[a + size_0][b + size_1] * img[i+a][j+b];
        }
    }
    img[i][j] = new_value;
}

int save_bitmap(const std::string& filename, Images& image_array)
{
    BITMAPINFO* BitmapInfo = image_array.get_Bitmapinfo();
    const size_t h = BitmapInfo->bmiHeader.biWidth;
    const size_t w = BitmapInfo->bmiHeader.biHeight;
    const int bits_per_pixel = BitmapInfo->bmiHeader.biBitCount;
    const size_t row_size = (bits_per_pixel * w + 31) / 32 * 4;

    byte* bitmapBytes = new byte[BitmapInfo->bmiHeader.biSizeImage];
    byte* writer = bitmapBytes;

    const size_t padding = row_size - w;

    for (size_t i = 0; i < h; ++i) {

        /* Przepisz wartosci pikseli wiersza obrazu. */
        for (size_t j = 0; j < w; j++) {
            *writer++ = image_array[h - i - 1][j];
        }

        /* Ustaw bajty wyrownania. */
        for (size_t j = 0; j < padding; j++) {
            *writer++ = 0;
        }
    }

    int status = SaveDIBitmap(filename.c_str(), BitmapInfo, bitmapBytes);
    delete[] bitmapBytes;

    return status;
}