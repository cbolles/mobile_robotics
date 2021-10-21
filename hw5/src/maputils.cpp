#include "hw5/maputils.hpp"

void setMapValue(nav_msgs::OccupancyGrid& map, int x, int y, int value) {
    map.data[map.info.width * x + y] = value;
}

int getMapValue(nav_msgs::OccupancyGrid& map, int x, int y) {
    return map.data[map.info.width * x + y];
}