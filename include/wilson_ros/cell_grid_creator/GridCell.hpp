//
// Created by aluquot on 16.08.17.
//

#ifndef PROJECT_GRIDCELL_HPP
#define PROJECT_GRIDCELL_HPP

#include <geometry_msgs/Point.h>

class GridCell {
public:
    GridCell(const geometry_msgs::Point &centerOriginal, double cellHeight, double cellWidth, double paddingHeight,
             double paddingWidth, GridCell *bottom, GridCell *left);

    void setNeighbours(GridCell *top, GridCell *right, GridCell *bottom, GridCell *left);

    void setNeighbours(GridCell *top, GridCell *right);

    void activateWith(geometry_msgs::Point p);

    bool isActivated() const;

    const geometry_msgs::Point &getCenterOriginal() const;

    const geometry_msgs::Point &getCenterBestMatch() const;

    const geometry_msgs::Point &getUpperBound() const;

    const geometry_msgs::Point &getLowerBound() const;

    GridCell *getNeigbourTop() const;

    GridCell *getNeigbourRight() const;

    GridCell *getNeigbourBootom() const;

    GridCell *getNeigbourLeft() const;

    void setNeighbourTop(GridCell *neighbourTop);

    void setNeighbourRight(GridCell *neighbourRight);

    void setNeighbourBootom(GridCell *neighbourBootom);

    void setNeighbourLeft(GridCell *neighbourLeft);

    bool cellContains(geometry_msgs::Point p);

private:

    double cellHeight;

    double cellWidth;

    bool activated;

    geometry_msgs::Point centerOriginal;

    geometry_msgs::Point centerBestMatch;

    double bestDistance;

    geometry_msgs::Point upperBound;

    geometry_msgs::Point lowerBound;

    GridCell *neighbourTop;

    GridCell *neighbourRight;

    GridCell *neighbourBootom;

    GridCell *neighbourLeft;
};


#endif //PROJECT_GRIDCELL_HPP
