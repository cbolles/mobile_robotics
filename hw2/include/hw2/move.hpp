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
 * Check to see if a robot is at the given point within tolerance.
 *
 * @param[in] currentPoint The current position of the robot
 * @param[in] targetPoint The target point of the robot
 * @return True if the robot is at the target point within tolerance
 */
bool atPoint(geometry_msgs::Point &currentPoint, geometry_msgs::Point &targetPoint);

/**
 * Check to see if the robot is pointing at a target within error.
 *
 * @param[in] heading The Z axis rotation of the robot in radians
 * @param[in] targetPoint The target point the robot should be facing
 */
bool pointingAtTarget(float heading, geometry_msgs::Point &targetPoint);

/**
 * Calculate the velocity to have the robot turn towards a point. This will have no
 * x or y velocity and just an angular velocity components.
 *
 * @param[in] currentPose The current pose of the robot
 * @param[in] targetPoint The target position the robot should be facing
 * @param[ouy] outputTwist The calculated velocity the robot should have.
 */
void turnTowardsPoint(geometry_msgs::Pose &currentPose, geometry_msgs::Point &targetPoint,
        geometry_msgs::Twist &outputTwist);
