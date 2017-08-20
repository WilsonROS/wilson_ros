//
// Created by aluquot on 16.08.17.
//

#ifndef PROJECT_GRIDREASSIGNER_HPP
#define PROJECT_GRIDREASSIGNER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <wilson_ros/cell_grid_creator/GridCell.hpp>

class CellGridCreator {
public:
    CellGridCreator(double cellHeight, double cellWidth);

    std::vector<GridCell> createGrid(double originCellHeight, double originCellWith,
                                     std::vector<geometry_msgs::Point> cells);

private:
    double cellHeight;
    double cellWidth;

    double minX;
    double minY;
    double maxX;
    double maxY;

    int gridCellsX;
    int gridCellsY;

    void findBounds(std::vector<geometry_msgs::Point> *cells);

    std::vector<std::vector<GridCell>> prepareGrid(double originCellHeight, double originCellWith);

    void activateGridCells(std::vector<std::vector<GridCell>> *grid, std::vector<geometry_msgs::Point> *originCells);

    std::vector<GridCell> extractActivatedCells(std::vector<std::vector<GridCell>> *grid);
};


#endif //PROJECT_GRIDREASSIGNER_HPP
