//
// Created by aluquot on 14.08.17.
//

#include <wilson_ros/grid_reassigner/GridReAssigner.hpp>
#include "wilson_ros/ZoneGenRunner.hpp"

ZoneGenRunner::ZoneGenRunner() : gen(42, 42) {
    sub = nh.subscribe("/cells", 10, &ZoneGenRunner::callbackGrid, this);
    pubZoneCenter = nh.advertise<geometry_msgs::PoseArray>("/zone_centers", 100);
    pubZoneBestMatch = nh.advertise<geometry_msgs::PoseArray>("/zone_best_match", 100);
    pubZoneLowerBound = nh.advertise<geometry_msgs::PoseArray>("/zone_lower_bound", 100);
    pubZoneUpperBound = nh.advertise<geometry_msgs::PoseArray>("/zone_upper_bound", 100);
    pubNavigationData = nh.advertise<wilson_ros::NavigationData>("/navigation_data", 100);
}

void ZoneGenRunner::callbackGrid(const nav_msgs::GridCells &msg) {
    gen.generateZones(msg);

    if (msg.cells.size() > 2) {
        unsigned int cellHeight = 1, cellWidth = 1;
        GridReAssigner reAssigner(cellHeight, cellWidth);
        std::vector<GridCell> reAssignedCells = reAssigner.reassignGrid(msg.cell_height, msg.cell_width, msg.cells);

        std::vector<geometry_msgs::Point> relevantCellPoints;
        unsigned long cellCount = reAssignedCells.size();
        for (unsigned long i = 0; i < cellCount; i++) {
            relevantCellPoints.push_back(reAssignedCells[i].getCenterOriginal());
        }

        GridReAssigner zoneAssigner(5, 2);
        std::vector<GridCell> zones = zoneAssigner.reassignGrid(cellHeight, cellWidth, relevantCellPoints);

        geometry_msgs::PoseArray centerPositions;
        geometry_msgs::PoseArray bestMatchPositions;
        geometry_msgs::PoseArray lowerBoundPositions;
        geometry_msgs::PoseArray upperBoundPositions;
        wilson_ros::NavigationData navigationDataMsg;
        centerPositions.header = msg.header;
        bestMatchPositions.header = msg.header;
        lowerBoundPositions.header = msg.header;
        upperBoundPositions.header = msg.header;
        navigationDataMsg.header = msg.header;
        for (int i = 0; i < zones.size(); i++) {
            GridCell *zone = &zones[i];

            geometry_msgs::Pose centerPose;
            geometry_msgs::Pose bestMatchPose;
            geometry_msgs::Pose lowerBoundPose;
            geometry_msgs::Pose upperBoundPose;

            centerPose.position = zone->getCenterOriginal();
            bestMatchPose.position = zone->getCenterBestMatch();
            lowerBoundPose.position = zone->getLowerBound();
            upperBoundPose.position = zone->getUpperBound();

            centerPositions.poses.push_back(centerPose);
            bestMatchPositions.poses.push_back(bestMatchPose);
            lowerBoundPositions.poses.push_back(lowerBoundPose);
            upperBoundPositions.poses.push_back(upperBoundPose);

            wilson_ros::Zone zoneMsg;
            zoneMsg.center_pose.position = zone->getCenterOriginal();
            zoneMsg.target_pose.position = zone->getCenterBestMatch(); //TODO: fix for reachable pose
            navigationDataMsg.zones.push_back(zoneMsg);
        }
        pubZoneCenter.publish(centerPositions);
        pubZoneBestMatch.publish(bestMatchPositions);
        pubZoneLowerBound.publish(lowerBoundPositions);
        pubZoneUpperBound.publish(upperBoundPositions);

        double minX = zones[0].getLowerBound().x;
        double minY = zones[0].getLowerBound().y;
        unsigned long measurementCount = reAssignedCells.size();
        for (unsigned long i = 0; i < measurementCount; i++) {
            GridCell *cell = &reAssignedCells[i];
            geometry_msgs::Point cellCenter = cell->getCenterOriginal();

            //find matching zonezone->getCenterOriginal()
            for (unsigned long j = 0; j < zones.size(); j++) {
                GridCell *zone = &zones[j];

                if (zone->cellContains(cellCenter)) {
                    geometry_msgs::Pose target_pose;
                    target_pose.position = cell->getCenterBestMatch();
                    navigationDataMsg.zones[j].target_poses.push_back(target_pose);

                    geometry_msgs::Pose center_pose;
                    center_pose.position = cellCenter;;
                    navigationDataMsg.zones[j].center_poses.push_back(center_pose);
                    break;
                }
            }
        }
        ROS_INFO_STREAM("count zones: " << zones.size() << " - cells in first zone: "
                                        << navigationDataMsg.zones[0].target_poses.size() << " / "
                                        << navigationDataMsg.zones[0].center_poses.size());
        pubNavigationData.publish(navigationDataMsg);
    }
}
