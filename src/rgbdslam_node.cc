#include "visual_odometry.h"
#include "dataset.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "rgbdslam");
    ros::NodeHandle nh;

    ros::Publisher pub_pointcloud = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/rgbdslam/pointcloud", 1);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    std::shared_ptr<rgbd_slam::Dataset> dataset(
            new rgbd_slam::Dataset("/home/qin/Downloads/training/mannequin_face_2"));

    if (dataset->Initialize()) {
        std::shared_ptr<rgbd_slam::VisualOdometry> vo(
                new rgbd_slam::VisualOdometry(dataset->GetCamera()));

        assert(vo->Initialize() == true);
        int i = 1;

        while (i < dataset->Size()) {
            vo->AddFrame(dataset->NextFrame());
            i++;

            // create transform of the frame
            Sophus::SE3d pose = vo->GetCurrentFramePose();
//            LOG(INFO) << "pose:-------------------------------------------------\n" << pose.matrix();
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = "map";
            transformStamped.child_frame_id = "camera";
            transformStamped.transform.translation.x = pose.matrix()(3, 0);
            transformStamped.transform.translation.y = pose.matrix()(3, 1);
            transformStamped.transform.translation.z = pose.matrix()(3, 2);

            transformStamped.transform.rotation.x = pose.unit_quaternion().x();
            transformStamped.transform.rotation.y = pose.unit_quaternion().y();
            transformStamped.transform.rotation.z = pose.unit_quaternion().z();
            transformStamped.transform.rotation.w = pose.unit_quaternion().w();

//            LOG(INFO) << "transformStamped: -------------------------\n"
//                      << transformStamped.transform.rotation
//                      << transformStamped.transform.translation;

                    br.sendTransform(transformStamped);

            pub_pointcloud.publish(vo->GetPointCloud());
            ros::spinOnce();
        }

    }
    return 0;
}