//
// Created by aluquot on 14.08.17.
//

#ifndef PROJECT_ZONEGENERATOR_HPP
#define PROJECT_ZONEGENERATOR_HPP

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>

class ZoneGenerator {
public:
    ZoneGenerator(double robot_radius, double invalidation_radius) {}
    void generateZones(const nav_msgs::GridCells &msg);
};


#endif //PROJECT_ZONEGENERATOR_HPP
