/**
 * Provides a series of utilities for going from one point to another.
 */
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

/**
 * Maximum distance the robot is allowed to be from its target point in meters
 */
#define DISTANCE_TOLERANCE 0.1

/**
 * Maximum amount of error between angle to point and the actual angle.
 * ~10 degrees.
 */
#define ANGLE_TOLERANCE 0.174533


/**
 * Max speed of the robot
 */
#define MAX_SPEED 0.5

/**
 * Minimum speed of the robot
 */
#define MIN_SPEED 0.1

/**
 * Distance where speed starts to decrease from the max value
 */
#define SLOW_DOWN_DISTANCE 1


/**
 * Max angular speed of the robot
 */
#define MAX_ANGULAR_SPEED 0.5

/**
 * Minimum angular speed of the robot
 */
#define MIN_ANGULAR_SPEED 0.05

/**
 * Angular difference to target angle at which to slow down
 */
#define SLOW_DOWN_ANGLE 0.5

/**
 * Calculate the distance between two points
 *
 * @param point1 The first point
 * @param point2 The second point
 */
double calculateDistance(const geometry_msgs::Point &point1,
        const geometry_msgs::Point &point2);


/**
 * Determines the angle between two points.
 *
 * @param point1 The first point
 * @param point2 The second point
 */
double calculateAngle(const geometry_msgs::Point &point1,
        const geometry_msgs::Point &point2);

/**
 * Check to see if a robot is at the given point within tolerance.
 *
 * @param[in] currentPoint The current position of the robot
 * @param[in] targetPoint The target point of the robot
 * @return True if the robot is at the target point within tolerance
 */
bool atPoint(const geometry_msgs::Point &currentPoint, geometry_msgs::Point &targetPoint);

/**
 * Check to see if the robot is pointing at a target within error.
 *
 * TODO: Have a graduated error based on distance to point
 *
 * @param[in] currentPose The pose of the robot
 * @param[in] targetPoint The target point the robot should be facing
 */
bool pointingAtTarget(const geometry_msgs::Pose &currentPose,
        geometry_msgs::Point &targetPoint);

/**
 * Calculate the velocity to have the robot turn towards a point. This will have no
 * x or y velocity and just an angular velocity components.
 *
 * @param[in] currentPose The current pose of the robot
 * @param[in] targetPoint The target position the robot should be facing
 * @param[ouy] outputTwist The calculated velocity the robot should have.
 */
void turnTowardsPoint(const geometry_msgs::Pose &currentPose,
        geometry_msgs::Point &targetPoint, geometry_msgs::Twist &outputTwist);

/**
 * Move towards a point. The speed will have a relation to the distance from the point.
 *
 * For distances > SLOW_DOWN_DISTANCE, the robot will go at MAX_SPEED.
 * For distanced <= SLOW_DOWN_DISTANCE, the robot will move at distance / SLOW_DOWN_DISTANCE
 *  of the MAX_SPEED.
 *
 * @param[in] currentPose The current pose of the robot
 * @param[in] targetPoint The point the robot is moving towards
 * @param[out] outputTwist The calculated velocity the robot should have
 */
void moveTowardsPoint(const geometry_msgs::Pose &currentPose,
        geometry_msgs::Point &targetPoint, geometry_msgs::Twist &outputTwist);
