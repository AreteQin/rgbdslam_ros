#include "visual_odometry.h"
#include "dataset.h"
#include <ros/ros.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "rgbdslam");
    ros::NodeHandle nh;

    ros::Publisher pub_pointcloud =
            nh.advertise < pcl::PointCloud < pcl::PointXYZRGB > >
            ("/rgbdslam/pointcloud", 1);

    std::shared_ptr <rgbd_slam::Dataset> dataset(
            new rgbd_slam::Dataset("/home/qin/Ubuntu/mannequin_face_2"));

    if (dataset->Initialize()) {
        std::shared_ptr <rgbd_slam::VisualOdometry> vo(
                new rgbd_slam::VisualOdometry(dataset->GetCamera()));

        assert(vo->Initialize() == true);
        int i = 1;

        while (i < dataset->Size()) {
            vo->AddFrame(dataset->NextFrame());
            i++;
            pub_pointcloud.publish(vo->GetPointCloud());
            ros::spinOnce();
        }

    }
    return 0;
}