//
// Created by aluquot on 14.08.17.
//

#include <wilson_ros/cell_grid_creator/CellGridCreator.hpp>
#include "wilson_ros/zone_generator/ZoneGenerator.hpp"

ZoneGenerator::ZoneGenerator() {
    sub = nh.subscribe("/cells", 10, &ZoneGenerator::generateZones, this);
    pubCellBestMatch = nh.advertise<geometry_msgs::PoseArray>("/cell_best_match", 100);
    pubZoneBestMatch = nh.advertise<geometry_msgs::PoseArray>("/zone_best_match", 100);
    pubNavigationData = nh.advertise<wilson_ros::NavigationData>("/navigation_data", 100);
}

void ZoneGenerator::generateZones(const nav_msgs::GridCells &msg) {
    if (msg.cells.size() > 2) {
        unsigned int cellHeight = 1, cellWidth = 1;

        std::vector<GridCell> measurementGridCells = createMeasurementCellGrid(cellHeight, cellWidth, msg);
        std::vector<GridCell> zones = createZoneGrid(5, 2, cellHeight, cellWidth, measurementGridCells);

        prepareZoneNavigationData(zones);
        finalizeNavigationData(zones, measurementGridCells);
        ROS_INFO_STREAM("count zones: " << zones.size() << " - cells in first zone: "
                                        << navigationData.zones[0].target_poses.size() << " / "
                                        << navigationData.zones[0].center_poses.size());
        publishMessages(msg.header);
    }
}

std::vector<GridCell>
ZoneGenerator::createMeasurementCellGrid(unsigned int cellHeight, unsigned int cellWidth,
                                         nav_msgs::GridCells msg) {
    CellGridCreator cellGridCreator(cellHeight, cellWidth);

    return cellGridCreator.createGrid(msg.cell_height, msg.cell_width, msg.cells);
}

std::vector<GridCell> ZoneGenerator::createZoneGrid(unsigned int zoneHeight, unsigned int zoneWidth, double cellHeight,
                                                    double cellWidth, std::vector<GridCell> cells) {
    std::vector<geometry_msgs::Point> relevantCellPoints;
    CellGridCreator zoneCreator(zoneHeight, zoneWidth);

    unsigned long cellCount = cells.size();
    for (unsigned long i = 0; i < cellCount; i++) {
        GridCell *currentCell = &cells[i];

        relevantCellPoints.push_back(currentCell->getCenterOriginal());

        geometry_msgs::Pose cellBestMatchPose = createPose(currentCell->getCenterBestMatch());
        cellBestMatch.poses.push_back(cellBestMatchPose);
    }

    return zoneCreator.createGrid(cellHeight, cellWidth, relevantCellPoints);
}

void ZoneGenerator::prepareZoneNavigationData(std::vector<GridCell> zones) {
    for (int i = 0; i < zones.size(); i++) {
        GridCell *zone = &zones[i];

        //TODO: fix for reachable pose
        geometry_msgs::Pose zoneBestMatchPose = createPose(zone->getCenterBestMatch());
        zoneBestMatch.poses.push_back(zoneBestMatchPose);

        // prepare zone-data
        wilson_ros::Zone zoneMsg;
        zoneMsg.target_pose = zoneBestMatchPose;
        zoneMsg.center_pose = createPose(zone->getCenterOriginal());
        navigationData.zones.push_back(zoneMsg);
    }
}

void ZoneGenerator::finalizeNavigationData(std::vector<GridCell> zones, std::vector<GridCell> measurementGridCells) {
    unsigned long measurementCount = measurementGridCells.size();
    for (unsigned long i = 0; i < measurementCount; i++) {
        GridCell *measurementCell = &measurementGridCells[i];
        geometry_msgs::Point cellCenter = measurementCell->getCenterOriginal();

        for (unsigned long zoneIndex = 0; zoneIndex < zones.size(); zoneIndex++) {
            GridCell *zone = &zones[zoneIndex];

            // cell only belongs to one zone at a time
            if (zone->cellContains(cellCenter)) {
                populateNavigationData(zoneIndex, cellCenter, measurementCell->getCenterBestMatch());
                break;
            }
        }
    }
}

void
ZoneGenerator::populateNavigationData(unsigned long zone, geometry_msgs::Point cellCenter, geometry_msgs::Point cellBestMatch) {
    geometry_msgs::Pose target_pose = createPose(cellBestMatch);
    navigationData.zones[zone].target_poses.push_back(target_pose);

    geometry_msgs::Pose center_pose = createPose(cellCenter);
    navigationData.zones[zone].center_poses.push_back(center_pose);
}

geometry_msgs::Pose ZoneGenerator::createPose(geometry_msgs::Point p) {
    geometry_msgs::Pose pose;

    pose.orientation.w = 1;
    pose.position = p;

    return pose;
}

void ZoneGenerator::publishMessages(std_msgs::Header msgHeader) {
    cellBestMatch.header = zoneBestMatch.header = navigationData.header = msgHeader;

    pubCellBestMatch.publish(cellBestMatch);
    cellBestMatch.poses.clear();

    pubZoneBestMatch.publish(zoneBestMatch);
    zoneBestMatch.poses.clear();

    pubNavigationData.publish(navigationData);
    navigationData.zones.clear();
}
