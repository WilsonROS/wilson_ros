//
// Created by aluquot on 16.08.17.
//

#include "wilson_ros/cell_grid_creator/CellGridCreator.hpp"

CellGridCreator::CellGridCreator(double cellHeight, double cellWidth) {
    this->cellHeight = cellHeight;
    this->cellWidth = cellWidth;
}

std::vector<GridCell> CellGridCreator::createGrid(double originCellHeight, double originCellWith,
                                                  std::vector <geometry_msgs::Point> cells) {
    double quotientHeight = cellHeight / originCellHeight;
    double quotientWidth = cellWidth / originCellWith;
    if (cellHeight != (quotientHeight * originCellHeight) || cellWidth != (quotientWidth * originCellWith)) {
        ROS_ERROR_STREAM("Cell-size do not match. Ensure that the expected cell of " << cellHeight << " x " << cellWidth
                                                                                     << " (height x width) contain multiples of the given cell size.");
        throw std::invalid_argument("Cell-size do not match");
    }

    minX = 0;
    minY = 0;
    maxX = 0;
    maxY = 0;
    findBounds(&cells);

    std::vector<std::vector<GridCell>> newCells = prepareGrid(originCellHeight, originCellWith);

    activateGridCells(&newCells, &cells);

    return extractActivatedCells(&newCells);
}

void CellGridCreator::findBounds(std::vector<geometry_msgs::Point> *cells) {
    unsigned long count = cells->size();
    for (unsigned long i = 0; i < count; i++) {
        maxX = std::max(cells->at(i).x, maxX);
        maxY = std::max(cells->at(i).y, maxY);

        minX = std::min(cells->at(i).x, minX);
        minY = std::min(cells->at(i).y, minY);
    }

    ROS_INFO_STREAM("Bounds found! lower: (" << minX << ", " << minY << "), upper: (" << maxX << ", " << maxY << ")");
}

std::vector<std::vector<GridCell>> CellGridCreator::prepareGrid(double originCellHeight, double originCellWith) {
    double paddingWidth = originCellWith / 2.;
    double paddingHeight = originCellHeight / 2.;
    double offsetX = cellWidth / 2. - paddingWidth;
    double offsetY = cellHeight / 2. - paddingHeight;
    ROS_INFO_STREAM(
            "gridCellsX: " << gridCellsY << " - gridCellsY:" << gridCellsY << " - offsetX: " << offsetX
                           << " - offsetY: " << offsetY);
    gridCellsX = static_cast<int>((maxX - minX) / this->cellWidth) + 1;
    gridCellsY = static_cast<int>((maxY - minY) / this->cellHeight) + 1;

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
            GridCell newCell = GridCell(center, cellHeight, cellWidth, bottom, left);

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
    return newCells;
}

void CellGridCreator::activateGridCells(std::vector<std::vector<GridCell>> *grid,
                                        std::vector<geometry_msgs::Point> *originCells) {
    unsigned long measurementCount = originCells->size();
    for (unsigned long i = 0; i < measurementCount; i++) {
        geometry_msgs::Point *p = &originCells->at(i);

        // rounding down will get the expected index
        double diffX = p->x - minX;
        int indexX = static_cast<int>(floor(diffX / cellWidth));
        double diffY = p->y - minY;
        int indexY = static_cast<int>(floor(diffY / cellHeight));

        grid->at(indexX)[indexY].activateWith(*p);
    }
}

std::vector<GridCell> CellGridCreator::extractActivatedCells(std::vector<std::vector<GridCell>> *grid) {
    std::vector<GridCell> activatedCells;
    for (int i = 0; i < gridCellsX; i++) {
        for (int j = 0; j < gridCellsY; j++) {
            GridCell *currentCell = &grid->at(i)[j];
            if (currentCell->isActivated()) {
                activatedCells.push_back(*currentCell);
            }
        }
    }

    ROS_INFO_STREAM("Activated-Cells: " << activatedCells.size());
    return activatedCells;
}
