//
// Created by aluquot on 14.08.17.
//
#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include <wilson_ros/ZoneGenRunner.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ZoneGenerator");
    ZoneGenRunner runner;

    ros::Rate loop_rate(10);

    ROS_INFO("Zone-Generator started!");

    while (ros::ok()) {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
