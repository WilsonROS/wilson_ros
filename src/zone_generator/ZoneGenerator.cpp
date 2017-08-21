//
// Created by aluquot on 14.08.17.
//

#include <wilson_ros/cell_grid_creator/CellGridCreator.hpp>
#include "wilson_ros/zone_generator/ZoneGenerator.hpp"

ZoneGenerator::ZoneGenerator() {
    sub = nh.subscribe("/cells", 10, &ZoneGenerator::generateZones, this);
    pubCellBestMatch = nh.advertise<geometry_msgs::PoseArray>("/navigation_cell_best_match", 100);
    pubZoneBestMatch = nh.advertise<geometry_msgs::PoseArray>("/navigation_zone_best_match", 100);
    pubCellGrid = nh.advertise<nav_msgs::GridCells>("/navigation_cell_grid", 100);
    pubZoneGrid = nh.advertise<nav_msgs::GridCells>("/navigation_zone_grid", 100);
    pubNavigationData = nh.advertise<wilson_ros::NavigationData>("/navigation_data", 100);
}

void ZoneGenerator::generateZones(const nav_msgs::GridCells &msg) {
    if (msg.cells.size() > 2) {
        double cellHeight = 0.95, cellWidth = 0.95;

        std::vector<GridCell> measurementGridCells = createMeasurementCellGrid(cellHeight, cellWidth, msg);
        std::vector<GridCell> zones = createZoneGrid(4.75, 1.9, cellHeight, cellWidth, measurementGridCells);

        prepareZoneNavigationData(zones);
        finalizeNavigationData(zones, measurementGridCells);
        ROS_INFO_STREAM("count zones: " << zones.size() << " - cells in first zone: "
                                        << navigationData.zones[0].target_poses.size() << " / "
                                        << navigationData.zones[0].center_poses.size());
        publishMessages(msg.header);
    }
}

std::vector<GridCell>
ZoneGenerator::createMeasurementCellGrid(double cellHeight, double cellWidth,
                                         nav_msgs::GridCells msg) {
    CellGridCreator cellGridCreator(cellHeight, cellWidth);
    cellGrid.cell_height = cellHeight;
    cellGrid.cell_width = cellWidth;


    std::vector<GridCell> originalCellGrid = cellGridCreator.createGrid(msg.cell_height, msg.cell_width, msg.cells);
    std::vector<GridCell> clearedGrid;
    int maximumSmallCellCount = static_cast<int>((cellHeight / msg.cell_height) * (cellWidth / msg.cell_width));
    int minimumThreshold = static_cast<int>(maximumSmallCellCount * 0.8);
    ROS_INFO_STREAM("maximumCellCount: " << maximumSmallCellCount << " - minimumThreshold: " << minimumThreshold);

    unsigned long gridCellCount = originalCellGrid.size();
    for (int j = 0; j < gridCellCount; ++j) {
        GridCell *currentCell = &originalCellGrid[j];
        if (currentCell->getActivationCounter() >= minimumThreshold) {
            clearedGrid.push_back(*currentCell);
            ROS_INFO_STREAM("Activation-count: " << currentCell->getActivationCounter());
        }
    }

    return clearedGrid;
}

std::vector<GridCell> ZoneGenerator::createZoneGrid(double zoneHeight, double zoneWidth, double cellHeight,
                                                    double cellWidth, std::vector<GridCell> cells) {
    std::vector<geometry_msgs::Point> relevantCellPoints;
    CellGridCreator zoneCreator(zoneHeight, zoneWidth);
    zoneGrid.cell_height = zoneHeight;
    zoneGrid.cell_width = zoneWidth;

    unsigned long cellCount = cells.size();
    for (unsigned long i = 0; i < cellCount; i++) {
        GridCell *currentCell = &cells[i];

        geometry_msgs::Point cellCenterPoint = currentCell->getCenterOriginal();
        relevantCellPoints.push_back(cellCenterPoint);
        cellGrid.cells.push_back(cellCenterPoint);

        geometry_msgs::Pose cellBestMatchPose = createPose(currentCell->getCenterBestMatch());
        cellBestMatch.poses.push_back(cellBestMatchPose);
    }

    return zoneCreator.createGrid(cellHeight, cellWidth, relevantCellPoints);
}

void ZoneGenerator::prepareZoneNavigationData(std::vector<GridCell> zones) {
    for (int i = 0; i < zones.size(); i++) {
        GridCell *zone = &zones[i];

        //TODO: fix for reachable pose / point
        geometry_msgs::Pose zoneBestMatchPose = createPose(zone->getCenterBestMatch());
        zoneBestMatch.poses.push_back(zoneBestMatchPose);

        geometry_msgs::Point zoneCenterPoint = zone->getCenterOriginal();
        zoneGrid.cells.push_back(zoneCenterPoint);

        // prepare zone-data
        wilson_ros::Zone zoneMsg;
        zoneMsg.target_pose = zoneBestMatchPose;
        zoneMsg.center_pose = createPose(zoneCenterPoint);
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
    cellGrid.header = zoneGrid.header = msgHeader;

    pubCellBestMatch.publish(cellBestMatch);
    cellBestMatch.poses.clear();

    pubZoneBestMatch.publish(zoneBestMatch);
    zoneBestMatch.poses.clear();

    pubNavigationData.publish(navigationData);
    navigationData.zones.clear();

    pubCellGrid.publish(cellGrid);
    cellGrid.cells.clear();

    pubZoneGrid.publish(zoneGrid);
    zoneGrid.cells.clear();
}
