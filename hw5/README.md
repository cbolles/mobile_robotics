# Mapping

## Using the Tool

The mapping tool has support for command line arguments via argparse. So
`python3 mapper.py --help` will list the arguments. The user specifies the
sensors to use for mapping and can tweak some other variables such as the
standard deviation to use.

## Collected Results

In the `images/` folder are results for laser only, sonar only, and results
from using both at once.

Something interesting which can't be seen in the static images is that my
model handled a moving object fairly well. At one point during testing I
accidently had the robot run into on obstacle causing it to move around. In
the map visualizer I could see the model updating in real time representing
the object's motion which was cool.

I had the robot move around just by providing a series of interesting points
that had the robot explore most of the space. Some of the space was left
unexplored however. I did run the mapper using the safe goto which I've been
slowly improving after the assignement was due.

### General Process

In general, the mapping logic that I used matched what we had done in class. 
Scans (either laser or sonar) are processed by updating areas that are
detected as empty, then processing obstacles. Obstacles have a Gaussian
distribution of their probablistic locations which then impact the resulting
map.

For the sake of processing time, the area to consider for the Gaussian
distribution was kept fairly small. The result is that the map can update
faster, but the gradiant representing the probablistic location of obstacles is
not as gradual. I think this is a reasonable trade off considering the
hardware that this mapping algorithm will have to run on. If I were to go back
and work on this (which I might, this was very interesting to work on), I
would definetly spend more time experiementing with the cost/benefits of
some of the meta parameters that control how far out the probability should
be considered.

For calculating the free space, I used Bresenham's Line drawing algorithm.
I probably took too naive an approach when it comes to how I updated the
free space. A better approach would probably include a more graduated
algorithm for updating the probability that there isn't an obstacle in a given
space.

### Laser Scan

The laser scan works fairly well. The approach I took didn't deviate much
from the class discussion. It preformed to what I would consider to be a
reasonable level. I don't have anything too fairly interesting to report on
the laser scan usage in particular.

### Sonar Scan

For the sonar, originally I was going to take a more complex approach really
diving into the wedge shape that truely represents the sensor's scanning range.
However, after some tweaking I took a more simplistic approach of treating
the sonar similar to a scan. The results were fairly decent. The noticable
differences between the sonar and the lasar is the precision of the map. With
the sonar for example, a straight wall may look more curved due to the actual
cone shaped range of data that the sonar is reading across.

One improvement I did implement was representing the standard deviation in the
x direction as a function of the reading from the sonar itself. The idea
being, the further the obstacle is, the wider the range the sonar id reading
across due to the wedge shape nature of the sonar. Essentially, obstacles
that are closer to the robot can be identified with greater certainty of
their location. In the "sonar only image", this can be seen with the walls
on the bottom and left being darker then the ones on the top and right. That
is because during my testing I had the robot go closer to the left and bottom
walls then the top and right.


## Unrelated Tangent

When working on this I noticed a fair deal of the calculations that take place
could be parallelized. For a while I've been interested in GPU developement
and I'm curious how well some of these algorithms would fair when attempting
to leverage GPU based speedups. I think in situations where you have a single
sensor in isolation, it would work fairly well allowing the developer to 
essentially update a series of pixels at once. Where I think things start to
become more complicated is when dealing with the case where multiple sensors
impact a single pixel. Maybe different passes considering each sensor in
isolation, then merging the many resulting models together? Would that
actually improve runtime? That certainly then becomes memory intensive.
