# GOTO

This goto works by constantly changing the velocity of the robot based on the
change in position. So within the callback for the Odometry values, the angle
to the target and the distance is calculated and the Twist of the robot is
then updated accordingly.

When the robot is turning, it is not going forward, when it is going forward, 
it is not turning. So first the robot checks to see if it is pointing in the
right direction. If it isn't, then the angular velocity is updated while linear
is zero. If the heading is correct, then the robot will just have a linear
velocity. This results in a more jagged motion where the robot will spot moving
forward if it needs to change its angular velocity.

For the speed both angular and linear speed follow the same formula. If the
distance (either angular or linear) is over a threshold (1 meter for linear
and 0.5 radians for angular) the robot will move at its max speed. If the
distance is less then that threshold, the robot will move at some proportion
of its max speed. For example, if the robot is 0.5 meters from its goal, the
speed will follow the formula.

```
Speed = (distance / SLOW_DOWN_DISTANCE) * MAX_SPEED
```

The code works fairly well. It is good for moving for locations "realitively" nearby
since it uses a lot zero point turns. For larger distances, since the robot adjusts
angular and linear velocity separately, it takes more time compared to an approach
that changes both components at the same time. An "easy" improvment would be to
dynamically change the tolerance on the angle of the robot towards the goal based on
how far away it is. When the robot is farther away from the goal it should spend most
of its time moving forward, then when it gets closer it should adjust its angle more.
In general, some refactoring is in order, extracting common mathematical expressions,
making other functions more generic. Overall my approach didn't change much during
development, I wrote a skeleton of the algorithm with stubbed functions, and from there
simply filled in the functions. I did experiment a little with changing linear and
angular velocity at the same time. While it did technically work. It would make large
sweeping paths which took more time to execute then my original method, so I reverted.
Overall I'm pleased with the outcome.
