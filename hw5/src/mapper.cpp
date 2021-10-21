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

#define WIN_X 100
#define WIN_Y 100
#define WIN_SCALE 1

// The map itself
static nav_msgs::OccupancyGrid map;

// the last dimension here is to allow for RGB, though my code is
// only making grays...
static float drawMap[WIN_X][WIN_Y][3];

// don't try to draw before the window exists...
static int winReady = 0;

// don't try to draw while the map is being updated...
std::mutex mapMutex;

static void display() {

  ROS_INFO("display called");

  mapMutex.lock();

  // you can use other formats for the data, google glDrawPixels for details
  glDrawPixels(WIN_X,WIN_Y,GL_RGB,GL_FLOAT, drawMap);
  
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

    // here is where you update your map (and pixels if separate)
    // I am just making a slightly lighter square around the robot's
    // position, to make sure things are working. 
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

// Of course you will need sensor callbacks too.  You could of course
// do the map update there as well, but either way, be careful that
// your callback doesn't take longer than the amount of time between
// calls, or things will buffer up and get out of sync.  A completely
// separate map building function, outside of either callback, may be
// the best idea, but again make sure that your data is consistent - if
// your callback takes too long, you will need to throw out some data
// somewhere to keep up.

int main(int argc, char *argv[]) {

    ros::init(argc,argv,"mapper");
    ros::NodeHandle n;

    ros::Subscriber odosub = n.subscribe("/r1/odom",1000,odoCallback);
    // and sensor callbacks

    // OpenGL expects to occupy the main thread, so you can't spin() here.
    // this object creates a separate thread that spins on its own, forever.
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