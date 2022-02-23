#include "util/get_pixel_value.h"

double GetPixelValue(const cv::Mat &img, double x, double y)
{
    // boundary check
    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;
    if (x >= img.cols)
        x = img.cols - 1;
    if (y >= img.rows)
        y = img.rows - 1;
    uchar *data = &img.data[int(y) * img.step + int(x)]; // 定位到做对比的像素位置
    double xx = x - floor(x);
    double yy = y - floor(y);
    // 使用bilinear interpolation计算该位置的近似灰度
    return double(
        (1 - xx) * (1 - yy) * data[0] +
        xx * (1 - yy) * data[1] +
        (1 - xx) * yy * data[img.step] +
        xx * yy * data[img.step + 1]);
}