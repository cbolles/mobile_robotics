#!/usr/bin/python3
'''
Quick hack sonar simulator, using laser scan data

Data fairly unreliable since geometry is bogus (assuming one laser at
robot center).  Hard-coded to P3DX sonar locations.
Z. Butler, Feb 2016
'''
import rospy
from p2os_msgs.msg import SonarArray
from sensor_msgs.msg import LaserScan
import math

class SonarMaker:
    __slots__ = 'spub','sonar_angs','indices','lasttime','seq'

    def __init__(self,pub):
        self.spub = pub
        self.sonar_angs= [math.pi/2, math.pi*5/18, math.pi/6, math.pi/18, \
                              -math.pi/18, -math.pi/6, -math.pi*5/18, -math.pi/2]
        self.lasttime = 0
        self.indices = None
        self.seq = 0

    def convert(self,lasermsg):
        if self.indices is None:
            self.indices = []
            # precompute which indices to scan for each sonar
            # assumes same laser geometry for all messages
            # sonar is 30 degrees total width, 15 degrees half
            #   but since laser is back a bit, shrink it down to 10
            rospy.loginfo('laser message: min angle %f, increment %f, ranges %d', lasermsg.angle_min,lasermsg.angle_increment,len(lasermsg.ranges))
            halfwidth = (math.pi/18)/lasermsg.angle_increment
            for ang in self.sonar_angs:
                middle = (ang-lasermsg.angle_min)/lasermsg.angle_increment
                self.indices.append(((int)(middle-halfwidth),\
                                         (int)(middle+halfwidth)))
            rospy.loginfo('computed indices: %s',str(self.indices))

        # only spit a new message after 0.1 secs, like the real robot
        msgtm = lasermsg.header.stamp.secs+lasermsg.header.stamp.nsecs/1000000000
        if msgtm-self.lasttime < 0.09:
            return
        self.lasttime = msgtm

        # construct the sonar message
        smsg = SonarArray()
        smsg.header.seq = self.seq
        self.seq += 1
        smsg.header.stamp = lasermsg.header.stamp
        smsg.header.frame_id = lasermsg.header.frame_id
        smsg.ranges_count = 8
        smsg.ranges = []
        for bounds in self.indices:
            minread = lasermsg.range_max
            for ind in range(bounds[0],bounds[1]+1):
                if lasermsg.ranges[ind] < minread:
                    minread = lasermsg.ranges[ind]
            # subtract offset from laser range since sonars not at center
            smsg.ranges.append(minread-0.2)
        self.spub.publish(smsg)

def main():
    rospy.init_node("sonar_simulator")
    spub = rospy.Publisher('/r1/sonar',SonarArray,queue_size=1)
    conv = SonarMaker(spub)
    rospy.Subscriber('/r1/pseudosonar/scan',LaserScan,conv.convert)
    rospy.spin()

if __name__ == '__main__':
    main()
