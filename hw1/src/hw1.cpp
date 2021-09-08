#include "ros/ros.h"
#include "p2os_msgs/MotorState.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <string>

#define LOOP_RATE 10

int main(int argc, char **argv) {

        // Determine how long to run for
        if(argc < 2) {
                std::cout << "Require the time to run in seconds" << std::endl;
                return 1;
        }
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
        // TODO: Remove hard coded value
        const int max_count = LOOP_RATE * num_seconds;
        int loop_count = 0;
        geometry_msgs::Twist velocity;

        // Make sure the robot starts out not moving
        velocity.linear.x = 0;
        velocity.angular.z = 0;
        vel_pub.publish(velocity);

        while(ros::ok() && loop_count < max_count) {
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
