//
// Created by aluquot on 16.08.17.
//

#include "../../include/wilson_ros/grid_reassigner/GridReAssigner.hpp"
#include "wilson_ros/grid_reassigner/GridCell.hpp"

GridReAssigner::GridReAssigner(unsigned int cellHeight, unsigned int cellWidth) {
    this->cellHeight = cellHeight;
    this->cellWidth = cellWidth;
}

std::vector<GridCell> GridReAssigner::reassignGrid(double originCellHeight, double originCellWith, std::vector<geometry_msgs::Point> cells) {
    // early return if cell-sizes do not match
    double quotientHeight = cellHeight / originCellHeight;
    double quotientWidth = cellWidth / originCellWith;
    if (cellHeight != (quotientHeight * originCellHeight) || cellWidth != (quotientWidth * originCellWith)) {
        ROS_ERROR_STREAM("Cell-size do not match. Ensure that the expected cell of " << cellHeight << " x " << cellWidth
                                                                                     << " (height x width) contain multiples of the given cell size.");
        throw std::invalid_argument("Cell-size do not match");
    }

    // find bounds
    double minX = 0, minY = 0, maxX = 0, maxY = 0;

    unsigned long count = cells.size();
    for (unsigned long i = 0; i < count; i++) {
        maxX = std::max(cells[i].x, maxX);
        maxY = std::max(cells[i].y, maxY);

        minX = std::min(cells[i].x, minX);
        minY = std::min(cells[i].y, minY);
    }
    ROS_INFO_STREAM("Bounds found! lower: (" << minX << ", " << minY << "), upper: (" << maxX << ", " << maxY << ")");

    // prepare grid
    int gridCellsX = static_cast<int>((maxX - minX) / this->cellWidth) + 1;
    int gridCellsY = static_cast<int>((maxY - minY) / this->cellHeight) + 1;
    double paddingWidth = originCellWith / 2.;
    double paddingHeight = originCellHeight / 2.;
    double offsetX = cellWidth / 2. - paddingWidth;
    double offsetY = cellHeight / 2. - paddingHeight;
    ROS_INFO_STREAM(
            "gridCellsX: " << gridCellsY << " - gridCellsY:" << gridCellsY << " - offsetX: " << offsetX
                           << " - offsetY: " << offsetY);

    std::vector<std::vector<GridCell>> newCells;
    for (int i = 0; i < gridCellsX; i++) {
        std::vector<GridCell> inner;
        newCells.push_back(inner);
        for (int j = 0; j < gridCellsY; j++) {
            geometry_msgs::Point center;
            center.x = minX + (cellWidth * i) + offsetX;
            center.y = minY + (cellHeight * j) + offsetY;

            GridCell *left = (i == 0) ? nullptr : &newCells[i - 1][j];
            GridCell *bottom = (j == 0) ? nullptr : &newCells[i][j - 1];
            GridCell newCell = GridCell(center, cellHeight, cellWidth, paddingHeight, paddingWidth, bottom, left);

            if (left != nullptr) {
                left->setNeighbourRight(&newCell);
            }
            if (bottom != nullptr) {
                bottom->setNeighbourTop(&newCell);
            }
            newCells[i].push_back(newCell);
        }
    }

    ROS_INFO_STREAM("Grid-Cells: " << (gridCellsX * gridCellsY));
    // activate grid cells
    unsigned long measurementCount = cells.size();
    for (unsigned long i = 0; i < measurementCount; i++) {
        geometry_msgs::Point *p = &cells[i];

        //find matching new Cell
        double diffX = p->x - minX;
        int indexX = static_cast<int>(floor(diffX / cellWidth));
        double diffY = p->y - minY;
        int indexY = static_cast<int>(floor(diffY / cellHeight));

        //activate cell with point
        newCells[indexX][indexY].activateWith(*p);
    }

    // extract activated cells
    std::vector<GridCell> activatedCells;
    for (int i = 0; i < gridCellsX; i++) {
        for (int j = 0; j < gridCellsY; j++) {
            GridCell *currentCell = &newCells[i][j];
            if (currentCell->isActivated()) {
                activatedCells.push_back(*currentCell);
            }
        }
    }
    ROS_INFO_STREAM("Activated-Cells: " << activatedCells.size());
    return activatedCells;
}
