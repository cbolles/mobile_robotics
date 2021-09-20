#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"

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
     * Initiate the robot.
     * 
     * @param velocityPublisher The ROS topic publisher to control velocity
     */
    Robot(ros::Publisher& velocityPublisher);

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
    void setPose(const geometry_msgs::Pose& pose);

    /**
     * Set the most recent laser scan from the robot.
     * 
     * @param laserScan The most recent laser scan.
     */
    void setLaserScan(const sensor_msgs::LaserScan& laserScan);

    /**

    /**
     * Determine if the robot is currently pointing at the target within
     * tolerane.
     * 
     * @param point The point to check if the robot is point at
     * @return True if the robot is pointing at that point within tolerance.
     */
    bool isPointingAt(const geometry_msgs::Point& point);

    /**
     * Determine if the robot is at a given point within tolerance. This will
     * not consider the angle the robot is pointing at, just if the robot
     * is at a location.
     * 
     * @param point The point to check if the robot is at
     * @return True if the robot is at that point
     */
    bool isAtPoint(const geometry_msgs::Point& point);

    /**
     * Get the distance to a given point.
     * 
     * @param point The point to get the distance from
     * @return The distance in meters.
     */
    double getDistance(const geometry_msgs::Point& point);

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
    void turnTowardsPoint(const geometry_msgs::Point& point);

    /**
     * Adjust the robots velocity so that it is moving towards the given point.
     * This will not include any angular velocity.
     * 
     * This will also include a "smart velocity" where the linear velocity of
     * the robot will be determined by how close the robot is from the target.
     * 
     * @param point The point to move towards.
     */
    void moveTowardsPoint(const geometry_msgs::Point& point);

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
    void unsafeGoTo(const geometry_msgs::Point& point);

    /**
     * Determines if there is an obstacle directly in front of the robot.
     * An obstacles is defined as any space that the robot cannot drive
     * through.
     * 
     * The obstacle is also defined as being within a set distance from the
     * robot.
     * 
     * @return True if there is an obstancle within some distance from the robot
     */
    bool obstacleInWay();

    /**
     * "Safe goto". This will have the robot follow a naive approach of the
     * bug algorithm. Once a wall is reached, the robot will move along the
     * wall until it can move in a straight line again.
     * 
     * @param point The point to goto.
     */
    void saveGoTo(const geometry_msgs::Point& point);

    /**
     * Have the robot stop moving, both angularly and linearly
     */
    void stop();

private:
    /** The current pose of the robot */
    geometry_msgs::Pose pose;
    /** The most recent laserr scan from the robot */
    sensor_msgs::LaserScan laserScan;
    /** The publisher to use to set the velocity of the robot */
    ros::Publisher* velocityPublisher;

    /** The tolerance for what is considering point at (5 degrees) */
    static constexpr double ANGLE_TOLERANCE = 0.0872665;
    /** Tolerance for how close the robot needs to be to a point in meters */
    static constexpr double DISTANCE_TOLERANCE = 0.05;
    
    /** The minimum angular speed of the robot in radians/s */
    static constexpr double MIN_ANGULAR_SPEED = 0.05;
    /** Maximum angular speed of the robot in radians/s */
    static constexpr double MAX_ANGULAR_SPEED = 0.5;
    /** The angle at which to start slowing down */
    static constexpr double SLOW_DOWN_ANGLE = 0.5;
    
    /** The minimum linear speed of the robot in meters/s */
    static constexpr double MIN_SPEED = 0.1;
    /** Maximum linear speed of the robot in meters/s */
    static constexpr double MAX_SPEED = 0.5;
    /** The distance from a point to start slowing down */
    static constexpr double SLOW_DOWN_DISTANCE = 1;

    /** Distance away from the robot where objects become potential obstacles */
    static constexpr double MIN_OBSTACLE_DISTANCE = 0.5;

    /** Number of values in the laser scan */
    static constexpr int NUM_LASER_POINTS = 640;

};