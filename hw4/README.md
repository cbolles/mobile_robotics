# Safe GOTO

## Description

This "safe" goto works using a bug algorithm. Mainly the robot has three main
states that determine its motions. "FREE" for when there is no obstacle and
the robot can move in a direct path, "BUG" when the robot is envoking the bug
algorithm, and "STOP" for no motion. The robot essentially transitions between
those states based on the presence of obstacles. 

The robot starts in FREE mode then when an obstacle is detected using the
kinect, the robot transitions to BUG. Just before the transition, a line is
constructed from the robot's immediate position to the goal
after finding an obstacle and the current location is recorded, these are used 
by the bug algorithm. In BUG, the robot moves around the obstacle using the
sonar to keep itself a fixed distance from the obstacle. The robot exits the BUG
mode when it reached the line that was constructed just before entering BUG and
the robot has actually made progress towards the goal. Then the cycle continues
until the goal is reached, at which case the robot enters STOP.

The line is used to determine when the robot has reached a point where it
should attempt to move away from the obstacle. The added condition that the
robot has also had to have gotten closer to the goal after reaching the line
helps in situations where the robot may have had to go in a semi-circle to
skirt around an obstacle.

## Points of Failure

Right now the algorithm lacks precision with calculating the possibility of
an obstacle in the way. This causes a couple of issues. First, the robot
sometimes gets too close/crashes into the corner of obstacles when the
obstacle is near the periphery of the sensors. The other issue is that the
algorithm does not do a good job of navigating tight spaces. This is manily
because I added lots of padding to compensate from the lack of precisions.
Therefore the robot may not be great at say, getting through a door way.

## Possible Imprvements

One big improvement would be in the way the robot determines it is possible
to start moving towards the goal. Right now the robot keeps moving around
the obstacle until it essentially regains its bearings from when it first
reached the obstacle. This is not an efficent solution. Say for example
the robot is moving around a rectangle. When the robot gets to the other
side, it should attempt to move directly towards the goal again. However,
my algorithm will have the robot continue to move around the rectangle until
it re-aligns itself with that constructed line.

Another improvement would be in the precious of the distance calculations. This
would allow the robot to navigate tigher spaces better.

Looking towards the next assignments, more care should be taken in using the
sensor values. Right now the sensor values are taken as is. With sensor
noise, especially in the kinect, I forsee the robot detecting non-existant
obstacles. I think a more ideal approach may be to average sensor values over
some small period and use those averaged values in the algorithm, at least
to partially deminish the impact of noise.

## What Changed During Development

Originally I was planning on avoiding the sonar values. However, with the
limited FOV of the kinect, the robot would have had more jerky motions
when the bug algorithm was having the robot keep itself a fixed distance
from the obstacle. Also the addition of the state machine model of the
algorithm was adopted about half-way through development to seperate the
logic to make it easier to test, debug, and implement the different
components.