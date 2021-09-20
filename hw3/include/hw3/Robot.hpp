#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

/**
 * Representation of the functionality the robot has. Controls the robot via
 * ROS topics.
 * 
 * The robot will require its state (position, sensor values, etc) to be
 * updated externally in interrupts. However writting of state will take place
 * within the robot itself (velocity updating).
 * 
 * The general flow of operations is as follows
 *  1. Call backs async update the robot's state
 *  2. One of the robot's GOTO functions will be called at some interval.
 *     It is recomment that the interval just be calling the GOTO within one of
 *     the call backs that also updates the robot's state.
 *  3. Check to see if the robot has reaching the target point via. The
 *     `isAtPoint` function.
 */
class Robot {
public:
    /**
     * Get the current pose of the robot that is stored.
     *
     * @return The pose of the robot
     */
    const geometry_msgs::Pose& getPose();

    /**
     * Set the pose of the robot.
     * 
     * @param pose The pose of the robot to set.
     */
    void setPose(geometry_msgs::Pose& pose);

    /**

    /**
     * Determine if the robot is currently pointing at the target within
     * tolerane.
     * 
     * @param point The point to check if the robot is point at
     * @return True if the robot is pointing at that point within tolerance.
     */
    bool isPointingAt(const geometry_msgs::Point point);

    /**
     * Determine if the robot is at a given point within tolerance. This will
     * not consider the angle the robot is pointing at, just if the robot
     * is at a location.
     * 
     * @param point The point to check if the robot is at
     * @return True if the robot is at that point
     */
    bool isAtPoint(const geometry_msgs::Point point);

    /**
     * Adjust the robots velocity so that it is turning to the given point.
     * This will have the robot turn in place and the robot will not have a
     * forward velocity.
     * 
     * This will also include a "smart velocity" where the anglular velocity
     * of the robot will be determined by how close the robot is to pointing
     * at the target.
     * 
     * @param point The point to turn towards.
     */
    void turnToPoint(const geometry_msgs::Point point);

    /**
     * Adjust the robots velocity so that it is moving towards the given point.
     * This will not include any angular velocity.
     * 
     * This will also include a "smart velocity" where the linear velocity of
     * the robot will be determined by how close the robot is from the target.
     * 
     * @param point The point to move towards.
     */
    void moveTowardsPoint(const geometry_msgs::Point point);

    /**
     * "Unsafe goto". This will have the robot go towards a give point
     * assuming no obstacles are in the way.
     * 
     * This will not block, but mearly update the velocity (angular and linear)
     * of the robot to have the robot move towards a point. Therefore it is
     * important that the function is called fairly rapidly, in line with the
     * rate at which sensor data is coming in.
     * 
     * @param point The point to goto.
     */
    void goToUnsafe(cost geometry_msgs::Point point);

private:
    /** The current pose of the robot */
    geometry_msgs::Pose pose;

    /** The tolerance for what is considering point at (5 degrees) */
    static constexpr double ANGLE_TOLERANCE = 0.0872665;
    /** Tolerance for how close the robot needs to be to a point in meters */
    static constexpr double DISTANCE_TOLERANCE = 0.05;

}