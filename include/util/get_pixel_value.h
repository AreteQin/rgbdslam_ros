#ifndef GETPIXELVALUE_H
#define GETPIXELVALUE_H

#include "common_include.h"

// 得到3D点在图像中像素坐标后，获取该像素的灰度
double GetPixelValue(const cv::Mat &img, double x, double y);

#endif