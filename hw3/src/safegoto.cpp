#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "p2os_msgs/MotorState.h"
#include "p2os_msgs/SonarArray.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

#include "hw3/utils.hpp"
#include "hw3/Robot.hpp"

#include <iostream>
#include <vector>

/*******************************************************************************
 * Application wide constants
 ******************************************************************************/

/**
 * Rate at which the main loop runs, needs to higher since C++ requires
 * `spinOnce` to be called to update callbacks.
 */
constexpr int LOOP_RATE = 20;

/*******************************************************************************
 * Global state variables
 ******************************************************************************/

/** Represents if the robot is actively making it's way to points */
bool running = false;
/** The robot, representing both state and functionality */
Robot* robot;
/** The target points the robot has to reach */
std::vector<geometry_msgs::Point> targetPoints;
/** The current target point index */
int targetPointIndex = 0;

/*******************************************************************************
 * ROS topic callbacks
 ******************************************************************************/

/**
 * Handle incoming odometry values. This call back both updates the state of
 * the robot and handles prompting the robot to update its current velocity
 *
 * @param odo The new odometry of the robot
 */
void odoCallback(const nav_msgs::Odometry &odo) {
    // Update robot state
    robot->setPose(odo.pose.pose);

    if(!running) {
        // Make sure the robot isn't moving
        robot->stop();
        return;
    }

    geometry_msgs::Point* targetPoint = &targetPoints[targetPointIndex];

    // Check if the target point needs to be updated
    if(robot->isAtPoint(*targetPoint)) {
        std::cout << "Reached point number: " << +targetPointIndex << std::endl;

        std::cout << "Goal X: " << +targetPoint->x << " Goal Y: " << +targetPoint-> y <<
        std::endl;
        std::cout << "X Pos: " << +robot->getPose().position.x << " Y Pos: " <<
            +robot->getPose().position.y << std::endl;

        double error = robot->getDistance(*targetPoint);
        std::cout << "Error: " << +error << std::endl << std::endl;

        if(targetPointIndex >= targetPoints.size()) {
            running = false;
        }
        else {
            targetPointIndex++;
        }
    }

    // Otherwise update robot velocity
    else {
        robot->safeGoTo(*targetPoint);
    }
}

/**
 * This call back just updates the robot's state.
 */
void kinectCallback(const sensor_msgs::LaserScan &laserScan) {
    // Update robot state
    robot->setLaserScan(laserScan);
}

/**
 * This call back just updated the robot's state.
 */
void sonarCallback(const p2os_msgs::SonarArray& sonarArray) {
    robot->setSonarArray(sonarArray);
}

int main(int argc, char** argv) {
    // Get the file path of the points from the user
    if(argc < 2) {
        std::cout << "Require file of points to goto" << std::endl;
        return 1;
    }

    std::string filePath = argv[1];

    // Create the list of target points
    readInPoints(filePath, targetPoints);

    // Init ros
    ros::init(argc, argv, "safegoto");
    ros::NodeHandle n;

    // Setup publishers
    ros::Publisher motorEnablePublisher = n.advertise<p2os_msgs::MotorState>
        ("/cmd_motor_state", 10);
    ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>
        ("/r1/cmd_vel", 1000);
    
    // Subscribe to robot state
    ros::Subscriber sub = n.subscribe("/r1/odom", 1000, odoCallback);
    ros::Subscriber laserSub = n.subscribe("/r1/kinect_laser/scan", 1000,
        kinectCallback);
    ros::Subscriber sonarSub = n.subscribe("/r1/sonar", 1000, sonarCallback);

    // Setup robot
    robot = new Robot(velocityPublisher);

    // Enable motors
    p2os_msgs::MotorState state;
    state.state = 1;
    motorEnablePublisher.publish(state);

    // Setup loop
    ros::Rate loopRate(LOOP_RATE);

    running = true;

    ros::spin();

    robot->stop();
    std::cout << "Finished" << std::endl;

    return 0;
}