// Mapping skeleton code
// Works OK with ROS, graphic updates are infrequent for me for some reason
// Probably not the right way to do it, please advise if you know better
// Zack Butler, Fall 2021, with guidance from previous anonymous students
// (please put your name in your code!)

#include <mutex>
#include <GL/glut.h>
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "hw5/maputils.hpp"

#include <unistd.h>

#include <iostream>
#include <vector>

#define WIN_X 100
#define WIN_Y 100
#define WIN_SCALE 1

// The map itself
static nav_msgs::OccupancyGrid map;

// Visual representation of map
static float drawMap[WIN_X][WIN_Y][3];

// don't try to draw before the window exists...
static int winReady = 0;

// don't try to draw while the map is being updated...
std::mutex mapMutex;

static void display() {

  ROS_INFO("display called");

  mapMutex.lock();

  // you can use other formats for the data, google glDrawPixels for details
  glDrawPixels(WIN_X,WIN_Y, GL_RGB, GL_FLOAT, drawMap);
  
  glFlush();

  mapMutex.unlock();
  
  glutSwapBuffers();
  
}

void odoCallback(const nav_msgs::Odometry::ConstPtr& odoMsg) {
    // coordinates are super-confusing - OpenGL actually uses right-handed
    // coord system for drawPixels, not left-handed as in most images
    // But Gazebo displays in left-handed.
    // So this should match reality but not Gazebo - to match Gazebo
    // you will need to invert Y.
    // (unless I got it all backwards somehow :( )
    int xpix = WIN_X*(odoMsg->pose.pose.position.x/WIN_SCALE) + WIN_X/2;
    int ypix = WIN_Y*(odoMsg->pose.pose.position.y/WIN_SCALE) + WIN_Y/2;

    mapMutex.lock();

    for (int r = ypix-10; r < ypix+10; r++) {
        for (int c = xpix-10; c < xpix+10; c++) {      
            if (getMapValue(map, r, c) > 10) {
                setMapValue(map, r, c, getMapValue(map, r, c) - 10);
                // To make gray, set R, G and B values to the same thing.
                // Again, this update doesn't look at all like what you will
                // want to do, especially if you are keeping odds in your map.
                drawMap[r][c][0] = (float)(100-getMapValue(map, r, c)) / 100;
                drawMap[r][c][1] = (float)(100-getMapValue(map, r, c)) / 100;
                drawMap[r][c][2] = (float)(100-getMapValue(map, r, c)) / 100;
            }
        }
    }
    mapMutex.unlock();

    // Tell OpenGL to please redraw the image when it gets around to it.
    if (winReady) glutPostRedisplay();
}

int main(int argc, char *argv[]) {
    bool isReal = true;

    // Check for optional value to distinguish between real and Gazebo
    if(argc == 2) {
        std::cout << "Running in simulation" << std::endl;
        isReal = false;
    }

    // Determine which topic to subscribe to
    std::string poseTopic = "/pose";
    std::string kinectTopic = "/scan";
    std::string sonarTopic = "/sonar";

    if(!isReal) {
        poseTopic = "/r1/odom";
        kinectTopic = "/r1/kinect_laser/scan";
        sonarTopic = "/r1/sonar";
    }

    ros::init(argc, argv, "mapper");
    ros::NodeHandle n;

    ros::Subscriber odosub = n.subscribe(poseTopic, 1, odoCallback);
    ros::Subscriber laserSub = n.subscribe(kinectTopic, 1,
        kinectCallback);
    ros::Subscriber sonarSub = n.subscribe(sonarTopic, 1, sonarCallback);

    // Start ROS spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // initialize the map (could be done in a more sensible spot)
    map.info.resolution = WIN_SCALE;
    map.info.width = WIN_X;
    map.info.height = WIN_Y;
    map.data.resize(WIN_X * WIN_Y);
    for( int x = 0; x < WIN_X; x++ ) {
        for( int y = 0; y < WIN_Y; y++ ) {
            setMapValue(map, x, y, 50);
        }
    }
    
    // Setup GUI
    glutInit( &argc, argv );
    glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE );
    glutInitWindowPosition( 50, 50 );
    glutInitWindowSize( WIN_X, WIN_Y );
    glutCreateWindow( "Map" );
    
    // OpenGL Callback
    glutDisplayFunc( display );

    winReady = 1;
    
    // this blocks, makes the window do its thing
    glutMainLoop();
    
    return 0;
}