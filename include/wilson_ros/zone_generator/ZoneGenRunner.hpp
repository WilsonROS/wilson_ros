//
// Created by aluquot on 14.08.17.
//

#ifndef PROJECT_ZONEGENRUNNER_HPP
#define PROJECT_ZONEGENRUNNER_HPP

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include "wilson_ros/zone_generator/ZoneGenerator.hpp"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <wilson_ros/NavigationData.h>

class ZoneGenRunner {
public:
    ZoneGenRunner();

private:
    ros::NodeHandle nh;
    ros::Publisher pubZoneCenter;
    ros::Publisher pubZoneBestMatch;
    ros::Publisher pubZoneUpperBound;
    ros::Publisher pubZoneLowerBound;
    ros::Publisher pubNavigationData;
    ros::Subscriber sub;

    ZoneGenerator gen;

    void callbackGrid(const nav_msgs::GridCells &msg);
};


#endif //PROJECT_ZONEGENRUNNER_HPP
