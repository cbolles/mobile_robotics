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
    double currentHeading = 2 * atan2(currentPose.orientation.z, currentPose.orientation.w);

    double angleDiff = fmod((angleBetween - currentHeading), (2 * M_PI));
    if(angleDiff < 0 && abs(angleDiff) > (2 * M_PI + angleDiff)) {
        angleDiff = 2 * M_PI + angleDiff;
    }
    else if(angleDiff > abs(angleDiff - 2 * M_PI)) {
        angleDiff = angleDiff - 2 * M_PI;
    }

    return abs(angleDiff) <= ANGLE_TOLERANCE;
}

void turnTowardsPoint(const geometry_msgs::Pose &currentPose,
        geometry_msgs::Point &targetPoint, geometry_msgs::Twist &outputTwist) {

    outputTwist.linear.x = 0;
    outputTwist.angular.z = 0;

    double angleBetween = calculateAngle(currentPose.position, targetPoint);
    double currentHeading = 2 * atan2(currentPose.orientation.z, currentPose.orientation.w);

    double angleDiff = fmod((angleBetween - currentHeading), (2 * M_PI));
    if(angleDiff < 0 && abs(angleDiff) > (2 * M_PI + angleDiff)) {
        angleDiff = 2 * M_PI + angleDiff;
    }
    else if(angleDiff > abs(angleDiff - 2 * M_PI)) {
        angleDiff = angleDiff - 2 * M_PI;
    }

    double speed = MAX_ANGULAR_SPEED;
    if(abs(angleDiff) <= SLOW_DOWN_ANGLE) {
        double proportionalSpeed = abs(angleDiff) / SLOW_DOWN_ANGLE * MAX_ANGULAR_SPEED;
        speed = std::max(proportionalSpeed, MIN_ANGULAR_SPEED);
    }
    

    // TODO: Add graduated angular velocity control
    if(angleDiff < 0)
        outputTwist.angular.z = -speed;
    else
        outputTwist.angular.z = speed;
}

void moveTowardsPoint(const geometry_msgs::Pose &currentPose,
        geometry_msgs::Point &targetPoint, geometry_msgs::Twist &outputTwist) {

    
    double distanceToPoint = calculateDistance(currentPose.position, targetPoint);
    double speed = MAX_SPEED;

    if(distanceToPoint <= SLOW_DOWN_DISTANCE) {
        double proportionalSpeed = distanceToPoint / SLOW_DOWN_DISTANCE * MAX_SPEED;
        speed = std::max(proportionalSpeed, MIN_SPEED);
    }
    

    outputTwist.linear.x = speed;
    outputTwist.angular.z = 0;
}
