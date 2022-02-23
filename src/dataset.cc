#include "dataset.h"
#include "frame.h"

#include <fstream> // to read the content of text files
#include <opencv2/opencv.hpp>

void tokenize(std::string const &str, const char delim,
              std::vector<std::string> &out)
{
    size_t start;
    size_t end = 0;

    while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
    {
        end = str.find(delim, start);
        out.push_back(str.substr(start, end - start));
    }
}

namespace rgbd_slam
{

    Dataset::Dataset(const std::string &dataset_path) : dataset_path_(dataset_path) {}

    bool Dataset::Initialize()
    {

        // read camera intrinsics
        std::ifstream dataset_path(dataset_path_ + "/calibration.txt");

        if (!dataset_path)
        {
            LOG(ERROR) << "cannot find " << dataset_path_ << "/calibration.txt";
            return false;
        }

        double K[4];
        for (int i = 0; i < 4; ++i)
        {
            dataset_path >> K[i];
        }

        camera_ = std::shared_ptr<Camera>(new Camera(K[0], K[1], K[2], K[3]));
        dataset_path.close();

        // read images
        std::ifstream image_stamps(dataset_path_ + "/associated.txt");
        int num_image_pairs = 0;
        if (!image_stamps)
        {
            // LOG(ERROR) << "cannot find " << dataset_path_ << "/associated.txt";
            std::cout << "cannot find " << dataset_path_ << "/associated.txt"<<std::endl;
            return false;
        }
        else // file exists
        {
            std::string line;
            while (getline(image_stamps, line))
            {
                std::vector<std::string> each_in_line;
                tokenize(line, ' ', each_in_line);
                depth_images_.push_back(cv::imread((dataset_path_ + "/" + each_in_line[3]), cv::IMREAD_UNCHANGED));
                color_images_.push_back(cv::imread((dataset_path_ + "/" + each_in_line[1]), cv::IMREAD_COLOR));
                // std::cout <<(dataset_path_ + "/" + each_in_line[3])<<std::endl;
            }
        }
        image_stamps.close();
        LOG(INFO) << depth_images_.size() << " pairs of images found." << std::endl;

        return true;
    }
    std::shared_ptr<Frame> Dataset::NextFrame()
    {
        auto new_frame = Frame::CreateFrame();
        new_frame->color_image_ = color_images_[current_image_index_];
        new_frame->depth_image_ = depth_images_[current_image_index_];
        current_image_index_++;
        // std::cout << current_image_index_ << " loop " << std::endl;
        return new_frame;
    }
}