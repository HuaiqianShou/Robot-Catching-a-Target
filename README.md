# Robot-Catching-a-Target

## Overview

Applied Dijkstra and 3D weighted A star algorithm that allows a point robot to catch a moving object. During
execution, the planner will be given an 8-connected 2D gridworld (that is, the robot can only move by at most one
unit along the X and/or Y axis). Each cell in the gridworld will be associated with the cost of moving through it.
This cost will be a positive integer. For each map, there is an associated collision threshold specified for the
planner.
Any cell with cost greater than or equal to this threshold is to be considered an obstacle that the robot cannot
traverse. The gridworld will be of size M by N, with the biggest gridworld in this homework being around 2,000 x
2,000 units.
The planner will also be given the start position of the robot, and the trajectory of the moving object as a sequence
of positions (for example: (2,3), (2,4), (3,4)). The object will also be moving on the 8-connected grid. The object
will move at the speed of one step per second.

## Getting Started

### Prerequisites
MATLAB

Compile /mex

### To Run
You can run matlab without the GUI using:
```sh
matlab -nodesktop
```

In MATLAB:

to compile the cpp code (Has to be run from MATLAB command window)
```sh
mex planner.cpp
```
to execute the planner on map1 (Has to be run from MATLAB command window)
```sh
runtest(‘map1.txt’)
```

