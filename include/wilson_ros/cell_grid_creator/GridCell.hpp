/**
 * \class GridCell
 *
 * \ingroup cell_grid_creator
 *
 * \brief Represents a cell of the grid created by CellGridCreator.
 */

#ifndef PROJECT_GRIDCELL_HPP
#define PROJECT_GRIDCELL_HPP

#include <geometry_msgs/Point.h>

class GridCell {
public:
    /**
     * \brief The constructor.
     *
     * @param centerOriginal
     *  The cell center position.
     * @param cellHeight
     *  The cell height.
     * @param cellWidth
     *  The cell width.
     * @param bottom
     *  Pointer to the cell's bottom neighbour.
     * @param left
     *  Pointer to the cell's left neighbour.
     */
    GridCell(const geometry_msgs::Point &centerOriginal, double cellHeight, double cellWidth,
                 GridCell *bottom, GridCell *left);

    /**
     * \brief Setter for the neighbours of the cell.
     *
     * @param top
     *  Pointer to the cell's top neighbour.
     * @param right
     *  Pointer to the cell's right neighbour.
     * @param bottom
     *  Pointer to the cell's bottom neighbour.
     * @param left
     *  Pointer to the cell's left neighbour.
     */
    void setNeighbours(GridCell *top, GridCell *right, GridCell *bottom, GridCell *left);

    /**
     * \brief Setter for the neighbours of the cell.
     *
     * @param top
     *  Pointer to the cell's top neighbour.
     * @param right
     *  Pointer to the cell's right neighbour.
     */
    void setNeighbours(GridCell *top, GridCell *right);

    /**
     * \brief Activates the cell if the given point lies within.
     *
     * @param p
     *  The point with which the cell should be activated.
     */
    void activateWith(geometry_msgs::Point p);

    /**
     * \brief Denotes whether the cell is avtive.
     *
     * @return
     *  True if the cell is active, false otherwise.
     */
    bool isActivated() const;

    /**
     * \brief Gives the amount of activation-atempts for this cell.
     *
     * @return
     *  Activation counter
     */
    int getActivationCounter() const;

    /**
     * \brief Denotes whether the given point is contained in this cell.
     *
     * @param p
     *  The point to check for.
     *
     * @return
     *  True if the point is contained in this cell, false otherwise.
     */
    bool cellContains(geometry_msgs::Point p);

    /**
     * \brief Getter for the position at the center of the cell.
     *
     * @return
     *  The position.
     */
    const geometry_msgs::Point &getCenterOriginal() const;

    /**
     * \brief Getter for the position that is nearest to the center of the cell
     * selected from all activations.
     *
     * @return
     *  The position.
     */
    const geometry_msgs::Point &getCenterBestMatch() const;

    /**
     * \brief Getter for the position of the lower left corner of the cell.
     *
     * @return
     *  The position.
     */
    const geometry_msgs::Point &getUpperBound() const;

    /**
     * \brief Getter for the position of the upper right corner of the cell.
     *
     * @return
     *  The position.
     */
    const geometry_msgs::Point &getLowerBound() const;

    /**
     * \brief Getter for the neighbours of the cell.
     *
     * @return
     *  Pointer to the cell's top neighbour.
     */
    GridCell *getNeigbourTop() const;

    /**
     * \brief Getter for the neighbours of the cell.
     *
     * @return
     *  Pointer to the cell's right neighbour.
     */
    GridCell *getNeigbourRight() const;

    /**
     * \brief Getter for the neighbours of the cell.
     *
     * @return
     *  Pointer to the cell's bottom neighbour.
     */
    GridCell *getNeigbourBottom() const;

    /**
     * \brief Getter for the neighbours of the cell.
     *
     * @return
     *  Pointer to the cell's left neighbour.
     */
    GridCell *getNeigbourLeft() const;

    /**
     * \brief Setter for the neighbours of the cell.
     *
     * @param neighbourTop
     *  Pointer to the cell's top neighbour.
     */
    void setNeighbourTop(GridCell *neighbourTop);

    /**
     * \brief Setter for the neighbours of the cell.
     *
     * @param neighbourRight
     *  Pointer to the cell's right neighbour.
     */
    void setNeighbourRight(GridCell *neighbourRight);

    /**
     * \brief Setter for the neighbours of the cell.
     *
     * @param neighbourBottom
     *  Pointer to the cell's bottom neighbour.
     */
    void setNeighbourBottom(GridCell *neighbourBottom);

    /**
     * \brief Setter for the neighbours of the cell.
     *
     * @param neighbourLeft
     *  Pointer to the cell's left neighbour.
     */
    void setNeighbourLeft(GridCell *neighbourLeft);

private:
    double cellHeight; /// Height of the grid cell

    double cellWidth; /// Width of the grid cell

    bool activated; /// Activation state of the cell

    int activationCounter; /// Count how many times a cell has been activated

    /**
     * The position at the center of the cell
     */
    geometry_msgs::Point centerOriginal;

    /**
     * The position that is nearest to the center of the cell
     */
    geometry_msgs::Point centerBestMatch;

    /**
     * The distance between the center position and the best matching position
     */
    double bestDistance;

    /**
     * The position of the lower left corner of the cell
     */
    geometry_msgs::Point upperBound;

    /**
     * The position of the upper right corner of the cell
     */
    geometry_msgs::Point lowerBound;

    GridCell *neighbourTop; /// Neighboring cell on the top

    GridCell *neighbourRight; /// Neighboring cell on the right

    GridCell *neighbourBottom; /// Neighboring cell on the bottom

    GridCell *neighbourLeft; /// Neighboring cell on the left
};


#endif //PROJECT_GRIDCELL_HPP
