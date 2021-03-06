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

/**
 * Calculate the distance between a point and a line.
 * 
 * @param point The point to calculate the distance for
 * @param line The line
 * @return The distance between them
 */
static double distancePointToLine(const geometry_msgs::Point& point,
    struct Line& line) {

    double numerator = abs(line.a * point.x + line.b * point.y + line.c);
    double denominator = sqrt(pow(line.a, 2) + pow(line.b, 2));

    return numerator / denominator;
}

/**
 * Check if a point is to the left of the robot, if not assume the point is
 * to the right of the robot.
 * 
 * @param pose The current pose of the bot
 * @param point The point in question
 * @return True if the point is to the left of the robot
 */
static bool pointToRight(const geometry_msgs::Pose& pose, const geometry_msgs::Point& point) {
    double angleBetween = calculateAngle(pose.position, point);
    double currentHeading = 2 * atan2(pose.orientation.z, pose.orientation.w);

    double angleDiff = fmod((angleBetween - currentHeading), (2 * M_PI));
    if(angleDiff < 0 && abs(angleDiff) > (2 * M_PI + angleDiff)) {
        angleDiff = 2 * M_PI + angleDiff;
    }
    else if(angleDiff > abs(angleDiff - 2 * M_PI)) {
        angleDiff = angleDiff - 2 * M_PI;
    }

    if(angleDiff < 0)
        return false;
    else
        return abs(angleDiff) >= 1.309;
}


/*******************************************************************************
 * Robot class implementation
 ******************************************************************************/
Robot::Robot(ros::Publisher& velocityPublisher, bool isReal) {
    this->velocityPublisher = &velocityPublisher;
    this->isReal = isReal;
    motionState = RobotMotionState::FREE_MOTION;
}

const geometry_msgs::Pose& Robot::getPose() {
    return pose;
}

void Robot::setPose(const geometry_msgs::Pose& pose) {
    this->pose = pose;
}

void Robot::setLaserScan(const sensor_msgs::LaserScan& laserScan) {
    this->laserScan = laserScan;
}

void Robot::setSonarArray(const p2os_msgs::SonarArray& sonarArray) {
    this->sonarArray = sonarArray;
}

Robot::Heading Robot::getPointDirection(const geometry_msgs::Point& point) {
    double angleBetween = calculateAngle(pose.position, point);
    double currentHeading = 2 * atan2(pose.orientation.z, pose.orientation.w);

    double angleDiff = fmod((angleBetween - currentHeading), (2 * M_PI));
    if(angleDiff < 0 && abs(angleDiff) > (2 * M_PI + angleDiff)) {
        angleDiff = 2 * M_PI + angleDiff;
    }
    else if(angleDiff > abs(angleDiff - 2 * M_PI)) {
        angleDiff = angleDiff - 2 * M_PI;
    }

    // Point headings in the physical world
    if(isReal) {
        if(abs(angleDiff) < ANGLE_TOLERANCE)
            return Heading::FORWARD;
        else if(angleDiff > 0)
            return Heading::RIGHT;
        else
            return Heading::LEFT;       // TODO: Add real world direction checking
    }
    // Point headings in the virtual world
    else {
        if(abs(angleDiff) < ANGLE_TOLERANCE)
            return Heading::FORWARD;
        else if(angleDiff > 0)
            return Heading::RIGHT;
        else
            return Heading::LEFT;
    }
    return Heading::FORWARD;
}

bool Robot::isAtPoint(const geometry_msgs::Point& point) {
    return calculateDistance(pose.position, point) <= DISTANCE_TOLERANCE;
}

void Robot::turnRight(float angularVelocity) {
    geometry_msgs::Twist twist;
    twist.angular.z = angularVelocity;
    velocityPublisher->publish(twist);
}

void Robot::turnLeft(float angularVelocity) {
    geometry_msgs::Twist twist;
    twist.angular.z = -angularVelocity;
    velocityPublisher->publish(twist);
}

void Robot::forward(float velocity) {
    geometry_msgs::Twist twist;
    twist.linear.x = velocity;
    velocityPublisher->publish(twist);
}

bool Robot::rightClear() {
    return sonarArray.ranges[7] > OBSTACLE_DISTANCE;
}

double Robot::getDistance(const geometry_msgs::Point& point) {
    return calculateDistance(pose.position, point);
}

void Robot::moveTowardsPoint(const geometry_msgs::Point& point) {
    geometry_msgs::Twist twist;

    double distanceToPoint = calculateDistance(pose.position, point);
    double speed = MAX_SPEED;

    if(distanceToPoint <= SLOW_DOWN_DISTANCE) {
        double proportionalSpeed = distanceToPoint / SLOW_DOWN_DISTANCE * MAX_SPEED;
        double min = MIN_SPEED;
        speed = std::max(proportionalSpeed, min);
    }
    
    twist.linear.x = speed;
    twist.angular.z = 0;

    velocityPublisher->publish(twist);
}

void Robot::unsafeGoTo(const geometry_msgs::Point& point) {
    // Check if we are alread at the point
    if(isAtPoint(point))
        return;

    switch(getPointDirection(point)) {
        case Heading::FORWARD:
            moveTowardsPoint(point);
            break;
        case Heading::RIGHT:
            turnRight(0.2);
            break;
        case Heading::LEFT:
            turnLeft(0.2);
            break;
    }
}

bool Robot::obstacleInWay() {
    // Since the number of points is known to be an even number, this is
    // assuming that no laser value represents directly in front of the robot.
    // Instead there will be laser values immediatly to the left and right
    // of the middle at LaserScan.angle_increment / 2 from straight ahead
    int rightIndex = NUM_LASER_POINTS / 2;
    int leftIndex = NUM_LASER_POINTS / 2 - 1;

    for(int i = 0; i < NUM_LASER_POINTS / 2; i++) {
        // Check to the left and right side, if either of them detect an
        // obstancle, assume there is an obstacle.
        // NOTE: This is problematic when dealinging with noisy sensors!
        float theta = 90 - (i * laserScan.angle_increment);
        if(laserScan.ranges[rightIndex - i] * sin(theta) < OBSTACLE_DISTANCE)
            return true;
        if(laserScan.ranges[leftIndex + i] * sin(theta) < OBSTACLE_DISTANCE)
            return true;
    }

    return false;
}

void Robot::freeMotionLogic(const geometry_msgs::Point& point) {
    // Check if the robot has reached the destination (to RobotMotionState::STOP)
    if(isAtPoint(point)) {
        motionState = RobotMotionState::STOP;
        return;
    }

    // Check if the robot has meet an obstacle (to RobotMotionState::BUG)
    if(obstacleInWay()) {
	    std::cout << "IN WAY" << std::endl;
        motionState = RobotMotionState::BUG;

        if(rightClear()) {
            bugDirection = Heading::RIGHT;
        } else {
            bugDirection = Heading::LEFT;
        }

        // Determine line from current point to target
        targetLine.a = point.y - pose.position.y;
        targetLine.b = pose.position.x - point.x;
        targetLine.c = (point.x - pose.position.x) * point.y +
            (pose.position.y - point.y) * point.x;

        previousPoint = pose.position;

        stop();
        return;
    }

    // Otherwise, keep on going
    unsafeGoTo(point);
}

void Robot::bugMotionLogic(const geometry_msgs::Point& point) {
    // Check if the robot has reached the destination
    if(isAtPoint(point)) {
        motionState = RobotMotionState::STOP;
        return;
    }

    // Check if we have to turn to the left or right to avoid the obstacle
    if (obstacleInWay()) {
        if(bugDirection == Heading::RIGHT) {
            turnRight(0.2);
        } else {
            turnLeft(0.2);
        }
    }
    else {
        // Check if we can start moving back towards goal
        if(getPointDirection(point) == Heading::FORWARD) {
            motionState = RobotMotionState::FREE_MOTION;
            return;
        }

        // Check if we have to angle back towards the obstacle
        if(bugDirection == Heading::RIGHT && sonarArray.ranges[1] > 0.3) {
            turnLeft(0.2);
        } else if(bugDirection == Heading::LEFT && sonarArray.ranges[6] > 0.3) {
            turnRight(0.2);
        } else {
            forward(0.2);
        }
    }
}

void Robot::safeGoTo(const geometry_msgs::Point& point) {
    switch(motionState) {
        case RobotMotionState::FREE_MOTION:
            freeMotionLogic(point);
            break;
        case RobotMotionState::BUG:
            bugMotionLogic(point);
            break;
        case RobotMotionState::STOP:
            if(!isAtPoint(point)) {
                motionState = RobotMotionState::FREE_MOTION;
            }
            stop();
        default:
            // Should not get here
            break;
    }
}

void Robot::stop() {
    geometry_msgs::Twist twist;
    velocityPublisher->publish(twist);
}
