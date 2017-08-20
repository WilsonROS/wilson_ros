//
// Created by aluquot on 16.08.17.
//

#include <ros/ros.h>
#include "wilson_ros/cell_grid_creator/GridCell.hpp"

GridCell::GridCell(const geometry_msgs::Point &centerOriginal, double cellHeight, double cellWidth,
                   GridCell *bottom, GridCell *left)
        : centerOriginal(centerOriginal) {
    this->cellHeight = cellHeight;
    this->cellWidth = cellWidth;
    this->neighbourBottom = bottom;
    this->neighbourLeft = left;

    this->centerBestMatch = centerOriginal;
    this->bestDistance = 0;
    this->activated = false;
    this->upperBound = geometry_msgs::Point();
    this->lowerBound = geometry_msgs::Point();
    this->neighbourTop = nullptr;
    this->neighbourRight = nullptr;

    double differenceX = cellWidth / 2;
    double differenceY = cellHeight / 2;
    this->upperBound.x = centerOriginal.x + differenceX;
    this->upperBound.y = centerOriginal.y + differenceY;
    this->lowerBound.x = centerOriginal.x - differenceX;
    this->lowerBound.y = centerOriginal.y - differenceY;
}

void GridCell::activateWith(geometry_msgs::Point p) {
    if (cellContains(p)) {
        double newDistance = hypot(this->centerOriginal.x - p.x, this->centerOriginal.y - p.y);

        if (this->activated && newDistance >= this->bestDistance) {
            return;
        }

        this->centerBestMatch = p;
        this->bestDistance = newDistance;
        this->activated = true;
    }
}

void GridCell::setNeighbours(GridCell *top, GridCell *right, GridCell *bottom, GridCell *left) {
    this->neighbourTop = top;
    this->neighbourRight = right;
    this->neighbourBottom = bottom;
    this->neighbourLeft = left;
}

void GridCell::setNeighbours(GridCell *top, GridCell *right) {
    this->neighbourTop = top;
    this->neighbourRight = right;
}

bool GridCell::isActivated() const {
    return activated;
}

const geometry_msgs::Point &GridCell::getCenterOriginal() const {
    return centerOriginal;
}

const geometry_msgs::Point &GridCell::getCenterBestMatch() const {
    return centerBestMatch;
}

const geometry_msgs::Point &GridCell::getUpperBound() const {
    return upperBound;
}

const geometry_msgs::Point &GridCell::getLowerBound() const {
    return lowerBound;
}

GridCell *GridCell::getNeigbourTop() const {
    return neighbourTop;
}

GridCell *GridCell::getNeigbourRight() const {
    return neighbourRight;
}

GridCell *GridCell::getNeigbourBottom() const {
    return neighbourBottom;
}

GridCell *GridCell::getNeigbourLeft() const {
    return neighbourLeft;
}

void GridCell::setNeighbourTop(GridCell *neighbourTop) {
    GridCell::neighbourTop = neighbourTop;
}

void GridCell::setNeighbourRight(GridCell *neighbourRight) {
    GridCell::neighbourRight = neighbourRight;
}

void GridCell::setNeighbourBottom(GridCell *neighbourBottom) {
    GridCell::neighbourBottom = neighbourBottom;
}

void GridCell::setNeighbourLeft(GridCell *neighbourLeft) {
    GridCell::neighbourLeft = neighbourLeft;
}

bool GridCell::cellContains(geometry_msgs::Point p) {
    bool matchX = (this->lowerBound.x <= p.x) && (this->upperBound.x >= p.x);
    bool matchY = (this->lowerBound.y <= p.y) && (this->upperBound.y >= p.y);
    return matchX && matchY;
}
