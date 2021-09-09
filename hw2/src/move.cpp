#include "hw2/move.hpp"

#include <math.h>

double calculateDistance(const geometry_msgs::Point &point1,
        const geometry_msgs::Point &point2){
    
    double xDiff = pow(point2.x - point1.x, 2);
    double yDiff = pow(point2.y - point1.y, 2);

    return sqrt(xDiff + yDiff);

}

double calculateAngle(const geometry_msgs::Point &point1,
        const geometry_msgs::Point &point2) {

    double xDiff = point2.x - point1.x;
    double yDiff = point2.y - point1.y;

    return atan2(yDiff, xDiff);
}

bool atPoint(const geometry_msgs::Point &currentPoint, geometry_msgs::Point &targetPoint) {
    return calculateDistance(currentPoint, targetPoint) <= DISTANCE_TOLERANCE;
}

bool pointingAtTarget(const geometry_msgs::Pose &currentPose,
        geometry_msgs::Point &targetPoint) {

    double angleBetween = calculateAngle(currentPose.position, targetPoint);
    double heading = 2 * atan2(currentPose.orientation.z, currentPose.orientation.w);

    return abs(heading - angleBetween) <= ANGLE_TOLERANCE;
}

void turnTowardsPoint(const geometry_msgs::Pose &currentPose,
        geometry_msgs::Point &targetPoint, geometry_msgs::Twist &outputTwist) {

    outputTwist.linear.x = 0;
    outputTwist.angular.z = 0;

    double angleBetween = calculateAngle(currentPose.position, targetPoint);
    double currentHeading = 2 * atan2(currentPose.orientation.z, currentPose.orientation.w);

    // TODO: Add graduated angular velocity control
    if(currentHeading > angleBetween)
        outputTwist.angular.z = -0.1;
    else
        outputTwist.angular.z = 0.1;
}

void moveTowardsPoint(const geometry_msgs::Pose &currentPose,
        geometry_msgs::Point &targetPoint, geometry_msgs::Twist &outputTwist) {

    // TODO: Add graduated linear velocity control
    outputTwist.linear.x = 0.1;
    outputTwist.angular.z = 0;
}
