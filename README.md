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
## Demo
Robot: Green

Target: Purple
### Map 1
![dda4fb7d83a8b0ecb05151de0ec18f4](https://user-images.githubusercontent.com/90527682/166162757-a0cdb6c0-a470-46e7-b8a4-be11d70149ba.png)
### Map 2
![1f13e6f79369ea7758645b2f197bcc1](https://user-images.githubusercontent.com/90527682/166162769-d4238e6e-6ac0-42ae-a6d0-89fd41f8cfe1.png)
### Map 3
![7d4d3ffd4a03f6c39913f97f2a57b0e](https://user-images.githubusercontent.com/90527682/166162777-98750df3-cc73-4d42-8a4a-506dc44a7b27.png)
### Map 4
![f587ba3565c9c84f607cc32bb431347](https://user-images.githubusercontent.com/90527682/166162781-0c6f6b68-ffd0-481f-b549-df294e2e4417.png)
### Map 5
![fed98cbc79fa503db865de8718f9e88](https://user-images.githubusercontent.com/90527682/166162787-cdab19e0-5301-44d3-b6ce-53e513300646.png)
### Map 6
![f0d56708fdd3efa43e6de0f1ca0b631](https://user-images.githubusercontent.com/90527682/166162790-44bd3d50-5eba-47ad-b75a-32a74e4bc57b.png)

