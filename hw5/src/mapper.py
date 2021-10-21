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

# a reasonable size? depends on the scale of the map and the
# size of the environment, of course:
MAPSIZE = 500

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

    def update_image(self):
        self.mapimage = ImageTk.PhotoImage(self.themap)       
        self.canvas.create_image(MAPSIZE/2, MAPSIZE/2, image = self.mapimage)

    def odds_to_pixel_value(self, odd):
        return (int(odd * 256),)

    # here I am putting the map update in the laser callback
    # of course you will need a sonar callback and odometry as well
    # actually, a completely separate function on a timer might be
    # a good idea, either way be careful if it takes longer to run
    # than the time between calls...
    def laser_update(self,lmsg):
        # note this function just lowers the odds in a random area... 
        sx = random.randint(100,500)
        sy = random.randint(100,500)
        rospy.loginfo('stomping on %d, %d',sx,sy)

        for x in range(sx-60,sx+60):
            for y in range(sy-60,sy+60):
                self.oddsvals[x][y] *= 0.9
                # you need to write a function that converts odds into
                # some appropriate grey-scale value
                self.mappix[x, y] = self.odds_to_pixel_value(self.oddsvals[x][y])

        # this puts the image update on the GUI thread, not ROS thread!
        # also note only one image update per scan, not per map-cell update
        self.after(0,self.update_image)    
        
def main():
    rospy.init_node("mapper")

    root = tk.Tk()
    m = Mapper(master=root,height=MAPSIZE,width=MAPSIZE)
    rospy.Subscriber("/r1/kinect_laser/scan",LaserScan,m.laser_update)
    
    # the GUI gets the main thread, so all your work must be in callbacks.
    root.mainloop()

if __name__ == "__main__":
    main()
