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
import math

# a reasonable size? depends on the scale of the map and the
# size of the environment, of course:
MAPSIZE = 100

MAPSCALE = 0.1

class Mapper(tk.Frame):    

    def __init__(self, *args, **kwargs):
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
        self.oddsvals = [[1.0 for _ in range(MAPSIZE)] for _ in range(MAPSIZE)]

        self.canvas = tk.Canvas(self,width=MAPSIZE, height=MAPSIZE)

        self.map_on_canvas = self.canvas.create_image(MAPSIZE/2, MAPSIZE/2, image = self.mapimage)
        self.canvas.pack()
        self.pack()

        self.pose = None
        self.pose_lock = False

    def update_image(self):
        self.mapimage = ImageTk.PhotoImage(self.themap)       
        self.canvas.create_image(MAPSIZE/2, MAPSIZE/2, image = self.mapimage)

    def odds_to_pixel_value(self, odd):
        return (int(odd * 256),)
    
    def odo_update(self, odo_msg):
        if not self.pose_lock:
            self.pose = odo_msg.pose.pose

    def laser_update(self, lmsg):
        if self.pose is None:
            return
        
        self.pose_lock = True
        heading = 2 * math.atan2(self.pose.orientation.z, self.pose.orientation.w) 
        if heading < 0:
            heading += 2 * math.pi
        
        for index, range_val in enumerate(lmsg.ranges):
            if math.isnan(range_val) or range_val == math.inf:
                continue
            
            angle_from_forward = lmsg.angle_min + index * lmsg.angle_increment

            full_angle = heading - angle_from_forward

            full_angle = full_angle % (2 * math.pi)

            x = int((self.pose.position.x + range_val * math.cos(full_angle)) / MAPSCALE + MAPSIZE / 2) 
            y = int((self.pose.position.y + range_val * math.sin(full_angle)) / MAPSCALE + MAPSIZE / 2)

            if x < 0 or y < 0 or x >= MAPSIZE or y >= MAPSIZE:
                return

            self.oddsvals[x][y] = 1
            self.mappix[x, y] = 256

        self.pose_lock = False
            
        # this puts the image update on the GUI thread, not ROS thread!
        # also note only one image update per scan, not per map-cell update
        self.after(0,self.update_image)    
        
def main():
    rospy.init_node("mapper")

    root = tk.Tk()
    m = Mapper(master=root,height=MAPSIZE,width=MAPSIZE)
    rospy.Subscriber("/r1/kinect_laser/scan", LaserScan, m.laser_update)
    rospy.Subscriber("/r1/odom", Odometry, m.odo_update)
    
    # the GUI gets the main thread, so all your work must be in callbacks.
    root.mainloop()

if __name__ == "__main__":
    main()
