//
// Created by qin on 30/12/23.
//

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>


std::string static_turtle_name;

int main(int argc, char **argv)
{
    ros::init(argc,argv, "my_static_tf2_broadcaster");

    static_turtle_name = "static_tf";
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "world";
    static_transformStamped.child_frame_id = static_turtle_name;
    static_transformStamped.transform.translation.x = 1;
    static_transformStamped.transform.translation.y = 1;
    static_transformStamped.transform.translation.z = 1;
    tf2::Quaternion quat;
    quat.setRPY(1, 1, 1);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);
    ROS_INFO("Spinning until killed publishing %s to world", static_turtle_name.c_str());
    ros::spin();
    return 0;
};