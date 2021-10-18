#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "p2os_msgs/SonarArray.h"

/**
 * Represents a line, used for detemining direction to target after the
 * robot started to bug.
 * 
 * Representation of the general form of a line
 */
struct Line {
    float a;
    float b;
    float c;
};

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
     * Heading values that can be used for directions
     */
    enum class Heading {
        FORWARD,
        RIGHT,
        LEFT
    };

    /**
     * Initiate the robot.
     * 
     * @param velocityPublisher The ROS topic publisher to control velocity
     * @param isReal Represents if the robot is in simulation or real
     */
    Robot(ros::Publisher& velocityPublisher, bool isReal);

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
     * Set the most recent sonar scan from the robot.
     * 
     * @param sonarArray The most recent sonar scan.
     */
    void setSonarArray(const p2os_msgs::SonarArray& sonarArray);

    /**
     * Get the direction the point is to the robot. (Forward, Right, Left)
     * 
     * For points that are in front of the robot, there is a tolerance that
     * is applied so that points that are within some angular range of
     * directly in front of the robot are considered in front of the robot.
     * 
     * @param point The point to check for
     * @return The direction the point is from the robot
     */
    Robot::Heading getPointDirection(const geometry_msgs::Point& point);

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
     * Have the robot turn right at some angular velocity
     *
     * @param angularVelocity The velocity for the robot to turn right at
     */
    void turnRight(float angularVelocity);

     /**
      * Have the robot turn left at some angular velocity
      *
      * @param angularVelocity The velocity for the robot to turn left at
      */
     void turnLeft(float angularVelocity);

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
    void safeGoTo(const geometry_msgs::Point& point);

    /**
     * Have the robot stop moving, both angularly and linearly
     */
    void stop();

private:
    /**
     * Represents the states of the robots motion logic. This allows for
     * a state machine implementation of the motion logic with easier means
     * to differentiate between how the robot is moving at a given time.
     */
    enum class RobotMotionState {
        /**
         * No obstacle in the robots way, can essentially move with
         * "unsafe goto" logic.
         * 
         * Transitions:
         *  TO BUG: Object detected
         *  TO STOP: Destination reached
         */
        FREE_MOTION,
        /**
         * Moving around an obstacle. This will move around the object until
         * it has reached a point where it can start moving back towards the
         * target.
         * 
         * Transitions:
         *  TO FREE_MOTION: Path unblocked
         *  TO STOP: Destination reached
         */
        BUG,
        /**
         * Not moving, has reached destination.
         */
        STOP
    };

    /** The current pose of the robot */
    geometry_msgs::Pose pose;
    /** The most recent laserr scan from the robot */
    sensor_msgs::LaserScan laserScan;
    /** The most recent sonar array scan from the robot */
    p2os_msgs::SonarArray sonarArray;
    /** The publisher to use to set the velocity of the robot */
    ros::Publisher* velocityPublisher;
    /** The state controlling the robot's motion logic */
    RobotMotionState motionState;
    /** Helper for bug algorithm */
    struct Line targetLine;
    /** Helper for previous points */
    geometry_msgs::Point previousPoint;
    /** Represents if the robot is in simulation */
    bool isReal;

    /**
     * Logic for free, un-obstructed motion, will also check for the need
     * for transitions
     * 
     * @param point The point the robot is trying to reach
     */
    void freeMotionLogic(const geometry_msgs::Point& point);

    /**
     * Logic for the bug algorithm, will also check for the need to 
     * transition
     * 
     * @param point The point the robot is trying to reach
     */
    void bugMotionLogic(const geometry_msgs::Point& point);

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
    static constexpr double MAX_SPEED = 0.3;
    /** The distance from a point to start slowing down */
    static constexpr double SLOW_DOWN_DISTANCE = 1;

    /** The width of the robot in meters */
    static constexpr double ROBOT_WIDTH = 0.5;
    /** The length of the robot in meters */
    static constexpr double ROBOT_LENGTH = 0.5;

    /** Distance away from the robot where objects become potential obstacles */
    static constexpr double OBSTACLE_DISTANCE = 0.5;

    /** Number of values in the laser scan */
    static constexpr int NUM_LASER_POINTS = 640;

};
