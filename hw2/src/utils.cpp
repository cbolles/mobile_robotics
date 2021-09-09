#include "hw2/utils.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>

void readInPoints(std::string &filePath, std::vector<geometry_msgs::Point> &points) {
    float x, y;
    std::string line;

    std::ifstream file;
    file.open(filePath);

    while(std::getline(file, line)) {
        std::istringstream in(line);
        geometry_msgs::Point point;
        in >> x >> y;

        point.x = x;
        point.y = y;
        point.z = 0;

        points.push_back(point);
    }

}
