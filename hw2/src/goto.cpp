#include "ros/ros.h"
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
geometry_msgs::Point &targetPoint;

/** The target point number (starting at zero) */
int targetPointNumber = 0;

/** Boolean representing if all points have been reached or not */
bool running = true;


/**
 * Callback to handle when a new location datapoint has been received. Updates
 * the velocity of the robot to adjust itself to the next target point.
 */
void odoCallback(const nav_msgs::Odometry &odo) {
    // If we aren't running, do nothin g
    if(!running) {
        return;
    }

    geometry_msgs::Pose &currentPose = odo.pose.pose; 

    // Check if we have reached the current target
    if(atPoint(currentPose.position, targetPoint)) {
        std::cout << "Reached point number: " << +targetPointNumber << std::endl;

        targetPointNumber++;

        // Check if we are done
        if(targetPointNumber >= targetPoints.size()) {
            running = false;
        }
        else {
            targetPoint = targetPoints[targetPointNumber];
        }
    }
    // Otherwise adjust velocity
    else {
        if(!pointingAtTarget(pose.orientation.z, targetPoint)) {
            turnTowardsPoint(currentPose, targetPoint,  
        }
    
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


        int num_seconds = std::stoi(argv[1]);
        std::cout << "Running for: " << +num_seconds << "s" << std::endl;

        ros::init(argc, argv, "talker");
        ros::NodeHandle n;

        // Publisher for enabling motors
        ros::Publisher motor_en_pub = n.advertise<p2os_msgs::MotorState>("/cmd_motor_state", 10);
        // Publisher for setting velocity
        ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/r1/cmd_vel", 1000);

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
        vel_pub.publish(velocity);

        while(ros::ok() && ) {
                velocity.linear.x = 0.01 * loop_count;
                velocity.angular.z = 0.01 * loop_count;

                vel_pub.publish(velocity);
                
                loop_count++;
                loop_rate.sleep();
        }

        velocity.linear.x = 0;
        velocity.angular.z = 0;
        vel_pub.publish(velocity);

        loop_rate.sleep();

        return 0;
}
