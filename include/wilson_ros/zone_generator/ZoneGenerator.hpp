//
// Created by aluquot on 14.08.17.
//

#ifndef PROJECT_ZONEGENRUNNER_HPP
#define PROJECT_ZONEGENRUNNER_HPP

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/PoseArray.h>

#include <wilson_ros/NavigationData.h>
#include <wilson_ros/zone_generator/ZoneGenerator.hpp>
#include <wilson_ros/cell_grid_creator/GridCell.hpp>

class ZoneGenerator {
public:
    ZoneGenerator();

private:
    ros::NodeHandle nh;
    ros::Publisher pubCellBestMatch;
    ros::Publisher pubZoneBestMatch;
    ros::Publisher pubNavigationData;
    ros::Subscriber sub;

    geometry_msgs::PoseArray cellBestMatch;
    geometry_msgs::PoseArray zoneBestMatch;
    wilson_ros::NavigationData navigationData;

    void generateZones(const nav_msgs::GridCells &msg);

    std::vector<GridCell>
    createMeasurementCellGrid(unsigned int cellHeight, unsigned int cellWidth, nav_msgs::GridCells msg);

    std::vector<GridCell>
    createZoneGrid(unsigned int zoneHeight, unsigned int zoneWidth, double cellHeight,
                   double cellWidth, std::vector<GridCell> cells);

    void prepareZoneNavigationData(std::vector<GridCell> zones);

    void finalizeNavigationData(std::vector<GridCell> zones, std::vector<GridCell> measurementGridCells);

    void
    populateNavigationData(unsigned long zone, geometry_msgs::Point cellCenter, geometry_msgs::Point cellBestMatch);

    geometry_msgs::Pose createPose(geometry_msgs::Point p);

    void publishMessages(std_msgs::Header msgHeader);
};


#endif //PROJECT_ZONEGENRUNNER_HPP
