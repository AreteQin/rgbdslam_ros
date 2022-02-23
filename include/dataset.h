#ifndef DATASET_H
#define DATASET_H
#include "camera.h"
#include "common_include.h"
#include "frame.h"

namespace rgbd_slam
{
    // read dataset
    // constructed with dataset path as input
    class Dataset
    {
    public:
        Dataset(const std::string &dataset_path);
        bool Initialize(); //return true if initialize successfully
        std::shared_ptr<Frame> NextFrame();
        std::shared_ptr<Camera> GetCamera() const // get color camera info
        {
            return camera_;
        }
        size_t Size(){
            return depth_images_.size();
        }

    private:
        std::string dataset_path_;
        int current_image_index_ = 0;
        std::shared_ptr<Camera> camera_;
        std::vector<cv::Mat> color_images_, depth_images_;
    };
}
#endif