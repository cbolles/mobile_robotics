#include <math.h>

#include "hw3/Robot.hpp"

#include "geometry_msgs/Twist.h"

/*******************************************************************************
 * Helper functions
 ******************************************************************************/
/**
 * Use the distance formula to find out how far away two points are
 * 
 * @param point1 The first point
 * @param point2 The second point
 * @return The distance between the two points
 */
static double calculateDistance(const geometry_msgs::Point &point1,
        const geometry_msgs::Point &point2){
    
    double xDiff = pow(point2.x - point1.x, 2);
    double yDiff = pow(point2.y - point1.y, 2);

    return sqrt(xDiff + yDiff);

}

/**
 * Calculate the angle between two points.
 * 
 * @param point1 The first point
 * @param point2 The second point
 * @return The angle between the two points in radians
 */
static double calculateAngle(const geometry_msgs::Point &point1,
    const geometry_msgs::Point &point2) {

    double xDiff = point2.x - point1.x;
    double yDiff = point2.y - point1.y;

    return atan2(yDiff, xDiff);
}


/*******************************************************************************
 * Robot class implementation
 ******************************************************************************/
Robot::Robot(ros::Publisher& velocityPublisher) {
    this->velocityPublisher = &velocityPublisher;
}

const geometry_msgs::Pose& Robot::getPose() {
    return pose;
}

void Robot::setPose(geometry_msgs::Pose& pose) {
    this->pose = pose;
}

bool Robot::isPointingAt(const geometry_msgs::Point& point) {
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

bool Robot::isAtPoint(const geometry_msgs::Point& point) {
    return calculateDistance(pose.position, point) <= DISTANCE_TOLERANCE;
}

double Robot::getDistance(const geometry_msgs::Point& point) {
    return calculateDistance(pose.position, point);
}

void Robot::turnToPoint(const geometry_msgs::Point& point) {
    geometry_msgs::Twist twist;

    double angleBetween = calculateAngle(pose.position, point);
    double currentHeading = 2 * atan2(pose.orientation.z, pose.orientation.w);

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

    if(angleDiff < 0)
        twist.angular.z = -speed;
    else
        twist.angular.z = speed;

    velocityPublisher->publish(twist);
}

void Robot::moveTowardsPoint(const geometry_msgs::Point& point) {
    geometry_msgs::Twist twist;

    double distanceToPoint = calculateDistance(pose.position, point);
    double speed = MAX_SPEED;

    if(distanceToPoint <= SLOW_DOWN_DISTANCE) {
        double proportionalSpeed = distanceToPoint / SLOW_DOWN_DISTANCE * MAX_SPEED;
        speed = std::max(proportionalSpeed, MIN_SPEED);
    }
    
    twist.linear.x = speed;
    twist.angular.z = 0;

    velocityPublisher->publish(twist);
}

void Robot::unsafeGoTo(cost geometry_msgs::Point& point) {
    // Check if we are alread at the pint
    if(isAtPoint(point))
        return;

    // Check if we need to angle towards the point
    if(!isPointingAt(point)) {
        turnTowardsPoint(point);
        return;
    }

    // Move towards point
    moveTowardsPoint(point);
}

void Robot::stop() {
    geometry_msgs::Twist twist;
    velocityPublisher->publish(twist);
}