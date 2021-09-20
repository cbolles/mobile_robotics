#include <math.h>

#include "hw3/Robot.hpp"

/*******************************************************************************
 * Helper functions
 ******************************************************************************/
static double calculateDistance(const geometry_msgs::Point &point1,
        const geometry_msgs::Point &point2){
    
    double xDiff = pow(point2.x - point1.x, 2);
    double yDiff = pow(point2.y - point1.y, 2);

    return sqrt(xDiff + yDiff);

}


/*******************************************************************************
 * Robot class implementation
 ******************************************************************************/
const geometry_msgs::Pose& Robot::getPose() {
    return pose;
}

void Robot::setPose(geometry_msgs::Pose& pose) {
    this->pose = pose;
}

bool isPointingAt(const geometry_msgs::Point& point) {
    double angleBetween = calculateAngle(pose.position, point);
    double currentHeading = 2 * atan2(pose.orientation.z, pose.orientation.w);

    double angleDiff = fmod((angleBetween - currentHeading), (2 * M_PI));
    if(angleDiff < 0 && abs(angleDiff) > (2 * M_PI + angleDiff)) {
        angleDiff = 2 * M_PI + angleDiff;
    }
    else if(angleDiff > abs(angleDiff - 2 * M_PI)) {
        angleDiff = angleDiff - 2 * M_PI;
    }

    return abs(angleDiff) <= ANGLE_TOLERANCE;
}

bool isAtPoint(const geometry_msgs::Point& point) {
    return calculateDistance(pose.position, point) <= DISTANCE_TOLERANCE;
}