#include <vector>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"

/**
 * From a given open file, read in all contained points into a list. The points
 * are expected in pairs in the form
 * ```
 * x1 y1
 * x2 y2
 * ```
 *
 * @param filePath The path to the file to read from
 * @param points The list of points to add onto
 */
void readInPoints(std::string &filePath, std::vector<geometry_msgs::Point> &points);
