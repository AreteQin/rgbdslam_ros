#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <opencv2/core/core.hpp>
#include <condition_variable> // used for multithreading
#include <atomic> // used for multithreading
#include <glog/logging.h> // also a dependency of ceres
#include "pcl_ros/point_cloud.h"
static const int DEPTH_SCALE = 512;
static const int MIN_DEPTH = 5;
static const int MAX_DEPTH = 250;
static const int MAX_OPTIMIZATION_ITERATION = 5;
static const double MAX_EUCLIDEAN_DISTANCE = 1.00005;
// unknown usage
// #include <typeinfo>
// #include <list>
// #include <map>
#endif