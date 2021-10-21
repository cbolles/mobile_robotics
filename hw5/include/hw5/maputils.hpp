/**
 * Series of utilities for interacting with the ros "OccupancyGrid"
 * as a two dimensional map of probabilities
 */
#include "nav_msgs/OccupancyGrid.h"


/**
 * Set the value of the map at the given x, y coordinate
 *
 * @param map The map to write to
 * @param x The x coordinate to set the value of
 * @param y The y coordinate to set the value of
 * @param value The value to write out
 */
void setMapValue(nav_msgs::OccupancyGrid& map, int x, int y, int value);

/**
 * Get the value of the map at the given x, y coordinate
 * 
 * @param map The map to read from
 * @param x The x coordinate to read from
 * @param y The y coordinate to read from
 * @return The value read from the map
 */
int getMapValue(nav_msgs::OccupancyGrid& map, int x, int y);