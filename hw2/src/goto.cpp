#include "ros/ros.h"
#include <ros/console.h>
#include "p2os_msgs/MotorState.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include "hw2/utils.hpp"
#include "hw2/move.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#define LOOP_RATE 10

/* Global variables for sharing state with callback */
/** List of points that need to be reached */
std::vector<geometry_msgs::Point> targetPoints;

/** The current target point */
geometry_msgs::Point *targetPoint = nullptr;

/** The target point number (starting at zero) */
int targetPointNumber = 0;

/** Boolean representing if all points have been reached or not */
bool running = false;

/** Publisher for setting velocity */
ros::Publisher velocityPublisher;


/**
 * Callback to handle when a new location datapoint has been received. Updates
 * the velocity of the robot to adjust itself to the next target point.
 */
void odoCallback(const nav_msgs::Odometry &odo) {
    // If we aren't running, do nothing
    if(!running) {
        geometry_msgs::Twist velocity;
        velocity.linear.x = 0;
        velocity.angular.z = 0;
        velocityPublisher.publish(velocity);
        return;
    }

    const geometry_msgs::Pose &currentPose = odo.pose.pose; 
    geometry_msgs::Twist outputTwist;

    // Check if we have reached the current target
    if(atPoint(currentPose.position, *targetPoint)) {
        std::cout << "Reached point number: " << +targetPointNumber << std::endl;

        std::cout << "Goal X: " << +targetPoint->x << " Goal Y: " << +targetPoint-> y <<
            std::endl;
        std::cout << "X Pos: " << +currentPose.position.x << " Y Pos: " <<
            +currentPose.position.y << std::endl;

        double error = calculateDistance(currentPose.position, *targetPoint);
        std::cout << "Error: " << +error << std::endl << std::endl;


        targetPointNumber++;

        // Check if we are done
        if(targetPointNumber >= targetPoints.size()) {
            ROS_DEBUG("All points reached, stopping");
            running = false;
        }
        else {
            targetPoint = &targetPoints[targetPointNumber];
        }
    }
    // Otherwise adjust velocity
    else {
        // Determine if the angle or direction should be changed
        if(!pointingAtTarget(currentPose, *targetPoint)) {
            turnTowardsPoint(currentPose, *targetPoint, outputTwist);
        } else {
            moveTowardsPoint(currentPose, *targetPoint, outputTwist);
        }
        ROS_DEBUG("Updating velocity");
        velocityPublisher.publish(outputTwist);
    }
}

int main(int argc, char **argv) {

        // Get the file path of the points from the user
        if(argc < 2) {
                std::cout << "Require file of points to goto" << std::endl;
                return 1;
        }

        std::string filePath = argv[1];

        // Create the list of target points
        readInPoints(filePath, targetPoints); 
        targetPoint = &targetPoints[0];


        ros::init(argc, argv, "talker");
        ros::NodeHandle n;

        // Publisher for enabling motors
        ros::Publisher motor_en_pub = n.advertise<p2os_msgs::MotorState>("/cmd_motor_state", 10);
        // Publisher for setting velocity
        velocityPublisher = n.advertise<geometry_msgs::Twist>("/r1/cmd_vel", 1000);

        // Subscribe to ODO data
        ros::Subscriber sub = n.subscribe("/r1/odom", 1000, odoCallback);

        // Enable motors
        p2os_msgs::MotorState state;
        state.state = 1;
        motor_en_pub.publish(state);

        // Setup loop
        ros::Rate loop_rate(LOOP_RATE);
        
        // Velocity control
        geometry_msgs::Twist velocity;

        // Make sure the robot starts out not moving
        velocity.linear.x = 0;
        velocity.angular.z = 0;
        velocityPublisher.publish(velocity);

        running = true;


        ros::spin(); 

        std::cout << "Finished" << std::endl;
        loop_rate.sleep();

        return 0;
}
