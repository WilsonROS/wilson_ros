/**
 * \class CellGridCreator
 *
 * \ingroup cell_grid_creator
 *
 * \brief Creates a new grid with uniform cells based on data from an existing
 *          grid.
 */

#ifndef PROJECT_GRIDREASSIGNER_HPP
#define PROJECT_GRIDREASSIGNER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <wilson_ros/cell_grid_creator/GridCell.hpp>

class CellGridCreator {
public:
    /**
     * \brief The constructor.
     *
     * @param cellHeight
     *  The cell height of the new grid.
     * @param cellWidth
     *  The cell width of the new grid.
     */
    CellGridCreator(double cellHeight, double cellWidth);

    /**
     * \breif Creates a new grid based on the given data & returns activated cells
     *          from it.
     *
     * @param originCellHeight
     *  The height of the given cells.
     * @param originCellWith
     *  The width of the given cells.
     * @param cells
     *  The cells of the existing grid.
     *
     * The grid created may cover areas which where not part of the previous
     * grid. Therefore the distinction between activated and not activated
     * cells is mad. Activated cells are such that cover at leas some area of
     * the previous grid.
     *
     * @return
     *  A list of activated grid cells.
     */
    std::vector<GridCell> createGrid(double originCellHeight, double originCellWith,
                                     std::vector<geometry_msgs::Point> cells);

private:
    double cellHeight; /// Height of each cell in the new grid

    double cellWidth;  /// Width of each cell in the new grid

    double minX; /// The lowest X value in the previous grid

    double minY; /// The lowest Y value in the previous grid

    double maxX; /// The highest X value in the previous grid

    double maxY; /// The highest Y value in the previous grid

    int gridCellsX; /// The amount of grid-cells in the X direction

    int gridCellsY; /// The amount of grid-cells in the X direction

    /**
     * \brief Find the upper and lower bounds of the previous grid and save
     *          them on the min* & max* fields.
     *
     * @param cells
     *  Pointer to the cell data of the previous grid.
     */
    void findBounds(std::vector<geometry_msgs::Point> *cells);

    /**
     * \brief Creates the data-structure for the new grid.
     *
     * @param originCellHeight
     *  The height of the given cells.
     * @param originCellWith
     *  The width of the given cells.
     *
     * @return
     *  A rectangular grid populated with inactive cells.
     */
    std::vector<std::vector<GridCell>> prepareGrid(double originCellHeight, double originCellWith);

    /**
     * \brief Activates the cells of the grid which overlap the previous grid.
     *
     * @param grid
     *  Pointer to the cell-data of the new grid.
     * @param originCells
     *  Pointer to the cell data of the previous grid.
     */
    void activateGridCells(std::vector<std::vector<GridCell>> *grid, std::vector<geometry_msgs::Point> *originCells);

    /**
     * \brief Creates a list of all the activated grid-cells from the new grid.
     *
     * @param grid
     *  Pointer to the cell-data of the new grid.
     *
     * @return
     *  The list of activated grid-cells.
     */
    std::vector<GridCell> extractActivatedCells(std::vector<std::vector<GridCell>> *grid);
};


#endif //PROJECT_GRIDREASSIGNER_HPP
