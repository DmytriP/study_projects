#include <cstdlib>
#include <functional>
#include <string>
#include <map>

#include "bitmap.h"
#include "improc.hpp"

std:: function<void(int, int, Images&)> neg = inverse;
std:: function<void(int, int, Images&)> filter = filtrate;

int main() {
    std::string img_root = "C:\\Users\\USER\\CLionProjects\\image_processor\\imgs\\";

    std::string input_filename = img_root + "lena.bmp";
    Images image_array = load_bitmap(input_filename);
    std::map<int,std:: function<void(int, int, Images&)>> queue;
    queue[1] = neg;
    queue[2] = filter;
    for (auto& i: queue)
    {
        transform(image_array, i.second);
    }
    save_bitmap(img_root + "out2.bmp", image_array);


    return EXIT_SUCCESS;
}
