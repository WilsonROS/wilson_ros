/**
 * \class ZoneGenerator
 *
 * \ingroup zone_generator
 *
 * \brief Creates and publishes navigation data based on created cells and
 *          zones.
 */

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
    ZoneGenerator(); /// The constructor

private:
    ros::NodeHandle nh; /// ROS node handler

    ros::Publisher pubCellBestMatch; /// Publisher for best match cell data

    ros::Publisher pubZoneBestMatch; /// Publisher for best match zone data

    ros::Publisher pubCellGrid; /// Publisher for the cell grid

    ros::Publisher pubZoneGrid; /// Publisher for the zone grid

    ros::Publisher pubNavigationData; /// Publisher for the navigation data

    ros::Subscriber sub; /// Subscriber for the coverage grid data

    /**
     * ROS message type for the best match cell data
     */
    geometry_msgs::PoseArray cellBestMatch;

    /**
     * ROS message type for the best match zone data
     */
    geometry_msgs::PoseArray zoneBestMatch;

    nav_msgs::GridCells cellGrid; /// ROS message type for the cell grid

    nav_msgs::GridCells zoneGrid; /// ROS message type for the zone grid

    /**
     * ROS message type for the navigation data
     */
    wilson_ros::NavigationData navigationData;

    /**
     * \brief Subscriber callback that generates and publishes navigation data
     *
     * @param msg
     *  The coverage grid data
     */
    void generateZones(const nav_msgs::GridCells &msg);

    /**
     * \brief Creates the measurement grid cell list.
     *
     * @param cellHeight
     *  The coverage grid cell height.
     * @param cellWidth
     *  The coverage grid cell weight.
     * @param msg
     *  The coverage grid cell positions.
     *
     * @return
     *  The list of grid-cells in the measurement grid.
     */
    std::vector<GridCell>
    createMeasurementCellGrid(double cellHeight, double cellWidth, nav_msgs::GridCells msg);

    /**
     * \brief Creates the zone grid cell list.
     *
     * @param zoneHeight
     *  The height of a zone.
     * @param zoneWidth
     *  The width of a zone.
     * @param cellHeight
     *  The measurement cell height.
     * @param cellWidth
     *  The measurement cell width.
     * @param cells
     *  The list of grid-cells in the measurement grid.
     *
     * @return
     *  The list of zones in the zone grid.
     */
    std::vector<GridCell>
    createZoneGrid(double zoneHeight, double zoneWidth, double cellHeight,
                   double cellWidth, std::vector<GridCell> cells);

    /**
     * \brief Populate the navigation data message with zone data.
     *
     * @param zones
     *  The list of zones in the zone grid.
     */
    void prepareZoneNavigationData(std::vector<GridCell> zones);

    /**
     * \brief Populates the navigation data message with measurement-cell data.
     *
     * @param zones
     *  The list of zones in the zone grid.
     * @param measurementGridCells
     *  The list of grid-cells in the measurement grid.
     */
    void finalizeNavigationData(std::vector<GridCell> zones, std::vector<GridCell> measurementGridCells);

    /**
     * \brief Add measurement data to the navigation data message.
     *
     * @param zone
     *  The index of the zone wo which the cell belongs.
     * @param cellCenter
     *  The center of the cell.
     * @param cellBestMatch
     *  The reachable position which is nearest to the center.
     */
    void
    populateNavigationData(unsigned long zone, geometry_msgs::Point cellCenter, geometry_msgs::Point cellBestMatch);

    /**
     * Creates a pose object from a point object.
     *
     * @param p
     *  The point object.
     *
     * @return
     *  The pose object.
     */
    geometry_msgs::Pose createPose(geometry_msgs::Point p);

    /**
     * \brief Publishes all messages via their respective handlers and resets
     * the messages properly.
     *
     * @param msgHeader
     *  The message header to use for all outgoing messages.
     */
    void publishMessages(std_msgs::Header msgHeader);
};


#endif //PROJECT_ZONEGENRUNNER_HPP
