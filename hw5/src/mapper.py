#!/usr/bin/python3
'''
  Some Tkinter/PIL code to pop up a window with a gray-scale
  pixel-editable image, for mapping purposes.  Does not run
  until you fill in a few things.

  Does not do any mapping.

  Z. Butler, 3/2016, updated 3/2018 and 3/2020
'''

import tkinter as tk
from PIL import Image, ImageTk
import random
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from p2os_msgs.msg import SonarArray
import math
from argparse import ArgumentParser
from enum import Enum
import numpy as np
import copy


class SensorSource(Enum):
    ALL = 'all'
    LASER = 'laser'
    SONAR = 'sonar'


# a reasonable size? depends on the scale of the map and the
# size of the environment, of course:
MAPSIZE = 100
# Scale of each pixel
MAPSCALE = 0.1
# The number of pixels to consider around an obstancle to apply the gaussian
# distribution
POINTS_TO_CHECK = 7

class Mapper(tk.Frame):    

    def __init__(self, sensor_source, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        self.master.title("I'm the map!")
        self.master.minsize(width=MAPSIZE,height=MAPSIZE)

        # makes a grey-scale image filled with 50% grey pixels
        # you can change the image type if you want color, check
        # the PIL (actually, Pillow) documentation
        self.themap = Image.new("L",(MAPSIZE,MAPSIZE),128)
        self.mapimage = ImageTk.PhotoImage(self.themap)

        # this gives us directly memory access to the image pixels:
        self.mappix = self.themap.load()
        # keeping the odds separately saves one step per cell update:
        self.oddsvals = [[0.5 for _ in range(MAPSIZE)] for _ in range(MAPSIZE)]

        self.canvas = tk.Canvas(self,width=MAPSIZE, height=MAPSIZE)

        self.map_on_canvas = self.canvas.create_image(MAPSIZE/2, MAPSIZE/2, image = self.mapimage)
        self.canvas.pack()
        self.pack()

        self.sensor_source = sensor_source

        self.new_pose = None
        self.pose = None
        self.new_laser = None
        self.laser = None
        self.sonar = None

        self.heading = 0
        self.new_heading = 0
        # Used to stop updating the input while the mapping is taking place
        self.lock_input = False

    def odds_to_pixel_value(self, odd):
        return int((1 - odd) * 256)

    def update_image(self):
        for x in range(0, MAPSIZE):
            for y in range(0, MAPSIZE):
                self.mappix[x, y] = self.odds_to_pixel_value(self.oddsvals[x][y])
        self.mapimage = ImageTk.PhotoImage(self.themap)       
        self.canvas.create_image(MAPSIZE/2, MAPSIZE/2, image = self.mapimage)
    
    def bresenham_open_update(self, x0, y0, x1, y1):
        """
        Update the area along the line as determined by the provided
        coordinates as detected as free space.

        :param x0: The starting x position
        :param y0: The starting y position
        :param x1: The end x position
        :param y1: The end y position
        """
        def line_low(x0, y0, x1, y1):
            dx = x1 - x0
            dy = y1 - y0
            yi = 1
            if dy < 0:
                yi = -1
                dy = -dy
            d = 2 * dy - dx
            y = y0

            for x in range(x0, x1 + 1):
                if x < 0 or y < 0 or x >= MAPSIZE or y >= MAPSIZE:
                    continue
                if self.oddsvals[x][y] > 0:
                    self.oddsvals[x][y] -= 0.1

                if d > 0:
                    y += yi
                    d += 2 * (dy - dx)
                else:
                    d += 2 * dy
        def line_high(x0, y0, x1, y1):
            dx = x1 - x0
            dy = y1 - y0
            xi = 1
            if dx < 0:
                xi = -1
                dx = -dx
            d = 2 * dx - dy
            x = x0

            for y in range(y0, y1 + 1):
                if x < 0 or y < 0 or x >= MAPSIZE or y >= MAPSIZE:
                    continue
                if self.oddsvals[x][y] > 0:
                    self.oddsvals[x][y] -= 0.1

                if d > 0:
                    x += xi
                    d += 2 * (dx - dy)
                else:
                    d += 2 * dx

        if abs(y1 - y0) < abs(x1 - x0):
            if x0 > x1:
                line_low(x1, y1, x0, y0)
            else:
                line_low(x0, y0, x1, y1)
        else:
            if y0 > y1:
                line_high(x1, y1, x0, y0)
            else:
                line_high(x0, y0, x1, y1)

    def apply_gaussian(self, mean_x, mean_y, std_dev_x, std_dev_y, rotation):
        """
        Apply a Gaussian distribution of odds for the existance of
        an object at a given location using the Markov's Assumption for
        the previous odds of an obstacle existing at a point.

        :param mean_x: The x coordinate to base the distribution around
        :param mean_y: The y coordinate to base the distribution around
        :param std_dev_x: The standard deviation of the reading in the x (not considering rotation)
        :param std_dev_Y: The standard deviation of the reading in the y (not considering rotation)
        :param rotation: The rotation of the robot
        """
        # Matrix of standard deviation assuming no rotation
        c_matrix = np.matrix([
                [pow(std_dev_x, 2), 0],
                [0, pow(std_dev_y, 2)]])
        # Rotation matrix to transform C
        rotation_matrix = np.matrix([
                [math.cos(rotation), -1 * math.sin(rotation)],
                [math.sin(rotation), math.cos(rotation)]])
        # Covariance relationship
        sigma = np.matmul(np.matmul(rotation_matrix, c_matrix), np.transpose(rotation_matrix))
        # Matrix representing the position of the center mean
        pos_mean_matrix = np.matrix([
                [mean_x],
                [mean_y]])
        
        # Iterate over the points to calculate the probability for about the mean
        start_x = int(mean_x - POINTS_TO_CHECK / 2)
        start_y = int(mean_y - POINTS_TO_CHECK / 2)
        end_x = int(mean_x + POINTS_TO_CHECK / 2)
        end_y = int(mean_y + POINTS_TO_CHECK / 2)

        for x in range(start_x, end_x + 1):
            for y in range(start_y, end_y + 1):
                if x < 0 or y < 0 or x >= MAPSIZE or y >= MAPSIZE:
                    continue
                pos_matrix = np.matrix([
                        [x],
                        [y]])
                
                probability_base = 1 / 2 * math.pi * np.sqrt(np.linalg.det(sigma))
                probability_exp = (-0.5 * np.transpose(pos_matrix - pos_mean_matrix)) * np.linalg.inv(sigma) * (pos_matrix - pos_mean_matrix)
                probability = np.power(probability_base, probability_exp)[0, 0]
                # TODO: Apply Markov's
                new_probability = (self.oddsvals[x][y] + probability) / 2
                self.oddsvals[x][y] = new_probability

    def laser_update_map(self):
        """
        Handle updating the map based on the laser scan information.
        """
        if self.pose is None or self.laser is None:
            return
         
        for index, range_val in enumerate(self.laser.ranges):
            # Check to make sure it is a valid input
            if math.isnan(range_val):
                continue

            # Update the clear space
            distance = range_val
            if range_val == math.inf:
                distance = self.laser.range_max
            
            # Determine x and y coordinate at the end of the scan
            angle_from_forward = self.laser.angle_min + index * self.laser.angle_increment
            full_angle = self.heading - angle_from_forward
            full_angle = full_angle % (2 * math.pi)

            x = self.pose.position.x + distance * math.cos(full_angle)
            y = self.pose.position.y + distance * math.sin(full_angle)

            # Scale the x and y and move the (0,0) to the center of the map
            x = int(x / MAPSCALE + MAPSIZE / 2)
            y = int(y / MAPSCALE + MAPSIZE / 2)

            x = min(x, MAPSIZE)
            y = min(y, MAPSIZE)

            r_x = int(self.pose.position.x / MAPSCALE + MAPSIZE / 2)
            r_y = int(self.pose.position.y / MAPSCALE + MAPSIZE / 2)
            
            free_x = self.pose.position.x + max(distance - 1, 0) * math.cos(full_angle)
            free_y = self.pose.position.y + max(distance - 1, 0) * math.sin(full_angle)
 
            self.bresenham_open_update(r_x, r_y, x, y)

            # Update around the obstacle with a Gaussian distribution
            if range_val < math.inf:
                # Bounds checking
                if x < 0 or y < 0 or x >= MAPSIZE or y >= MAPSIZE:
                    continue

                self.apply_gaussian(x, y, 10, 10, full_angle)
    
    def sonar_update_map(self):
        if self.pose is None or self.sonar is None:
            return

        # Loop over all the sonar ranges
        for range_val, angle in zip(self.sonar, [90, 50, 30, 10, -10, -30, -50, -90]):
            # Naive approach, TODO: Treat sonar output as cone
            
            # Check to make sure the input is valid
            if math.isnan(range_val) or range_val == math.inf:
                continue
            
            full_angle = self.heading - math.radians(angle)

            # Calculate the x and y position in the world coordinate system
            x = self.pose.position.x + range_val * math.cos(full_angle)
            y = self.pose.position.y + range_val * math.sin(full_angle)

            # Scale the positions and translate the positions such that the origin is in the center of the map
            x = int(x / MAPSCALE + MAPSIZE / 2)
            y = int(y / MAPSCALE + MAPSIZE / 2)


            # Bounds checking
            if x < 0 or y < 0 or x >= MAPSIZE or y >= MAPSIZE:
                continue

            self.oddsvals[x][y] = 1
            self.mappix[x, y] = 256 

    def update_map(self):
        """
        Handles updating the map. Will update the state of the map using either
        the sonar, laser, or both.

        The map will also not update if the sensor values are not present,
        or if the map is actively being updated.
        """
        if self.new_pose is None or self.sonar is None or self.lock_input:
            return

        # Lock input from coming in while mapping
        self.lock_input = True

        self.laser = copy.deepcopy(self.new_laser)
        self.pose = copy.deepcopy(self.new_pose)
        self.heading = self.new_heading

        if self.sensor_source == SensorSource.ALL:
            self.laser_update_map()
            self.sonar_update_map()
        elif self.sensor_source == SensorSource.LASER:
            self.laser_update_map()
        elif self.sensor_source == SensorSource.SONAR:
            self.sonar_update_map()
        
        # this puts the image update on the GUI thread, not ROS thread!
        # also note only one image update per scan, not per map-cell update
        self.after(0,self.update_image)    

        # Unlock input
        self.lock_input = False
 
    def odo_update(self, odo_msg):
        self.new_pose = odo_msg.pose.pose
        self.new_heading = 2 * math.atan2(self.new_pose.orientation.z, self.new_pose.orientation.w) 
        if self.new_heading < 0:
            self.new_heading += 2 * math.pi
    
    def sonar_update(self, sonar_msg):
        if not self.lock_input:
            self.sonar = sonar_msg.ranges
            self.update_map() 

    def laser_update(self, lmsg):
        self.new_laser = lmsg

                
def main():
    # Argument parsing
    parser = ArgumentParser('ROS mapper and visualizer')
    parser.add_argument('sensor', help='Sensors to use to build the map',
                        default='all', choices=['sonar', 'laser', 'all'])
    args = parser.parse_args()

    sensor_source = SensorSource.ALL
    if args.sensor == 'sonar':
        sensor_source = SensorSource.SONAR
    elif args.sensor == 'laser':
        sensor_source = SensorSource.LASER

    rospy.init_node("mapper")

    root = tk.Tk()
    m = Mapper(sensor_source, master=root,height=MAPSIZE,width=MAPSIZE)
    
    # ROS subscribers
    rospy.Subscriber("/r1/kinect_laser/scan", LaserScan, m.laser_update, queue_size=1)
    rospy.Subscriber("/r1/odom", Odometry, m.odo_update, queue_size=1)
    rospy.Subscriber("/r1/sonar", SonarArray, m.sonar_update, queue_size=1)
    
    # the GUI gets the main thread, so all your work must be in callbacks.
    root.mainloop()

if __name__ == "__main__":
    main()
