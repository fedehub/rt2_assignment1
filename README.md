# First Assignment of the Research Track 2 course - Action branch

## Table of contents 
- Index 
  - [List of the required packages](#list-of-the-required-packages)
  - [Description of the branch](#description-of-the-branch)
    - [Action folder](#action-folder)
    - [Script folder](#script-folder)
    - [Launch folder](#launch-folder)
    - [Srv folder](#srv-folder)
    - [Src folder](#src-folder)
    - [URDF folder](#urdf-folder)
  - [Rqt-graph (ROS tool)](#rqt-graph-ros-tool)
  - [Documentation](#documentation)
  - [How to run the code](#how-to-run-the-code)
  - [Release History](#release-history)
  - [Meta](#meta)
  - [Contributing](#contributing)
  
## List of the required packages

Within the project, by means of the [Actonlib package](http://wiki.ros.org/actionlib) it has been possible to create:
* servers able to  execute long-running tasks
* clients  interacting with them 
The *Action template* has been defined through an **.action structure** defined in ROS and placed inside the action folder of the  **rt2_assignment1** package. 
The action messages are generated automatically from the [GoalReaching.action](https://github.com/fedehub/rt2_assignment1/blob/action/action/GoalReaching.action) file-
This file defines:
1. the *type* and *format* of the goal
2. the *result* 
3. the *feedback topics* for the action.


## Description of the branch

The action branch, should contain the same package in ROS, but with the **go_to_point node** modelled as a **ROS action server**, instead of a “simple” server. Given that, the robot Finite State Machine (FSM) node should now implement mechanisms for possibly cancelling the goal, when the related user command is received.

This branch is structured as reported below, in the following paragraphs.

### Action folder 

It contains the instruction for defining the expected **action**. 

### Script folder

It contains:
1. [go_to_point](https://github.com/fedehub/rt2_assignment1/blob/action/scripts/go_to_point.py)
2. [user_interface](https://github.com/fedehub/rt2_assignment1/blob/action/scripts/user_interface.py): If user enter 1, the command  client calls the state machine with the start string (robot starts to move randomly). If it enters 0, in this branch we have defined a client for the action so that teh goal of the robot is canceled and the robot is stopped in the actual position through a twist message.
   
### Launch folder 

Inside this folder, there is  the [sim.launch file](https://github.com/fedehub/rt2_assignment1/blob/action/launch/sim.launch), responsible for the starting of  the overall simulation.


### Srv folder 

This latter, contains two custom services:
1. [Command.srv](https://github.com/fedehub/rt2_assignment1/blob/action/srv/Command.srv)
2. [RandomPosition.srv](https://github.com/fedehub/rt2_assignment1/blob/action/srv/RandomPosition.srv): 
   * As request it issues **two ranges** of intervals for the **x** and **y coordinates** 
   * As reply it provides a **random x,y** and **theta** **values**

### Src folder

1. [postion_service.cpp](https://github.com/fedehub/rt2_assignment1/blob/action/src/position_service.cpp): It simply returns the random position which the robot has to reach.
2. [state_machine.cpp](https://github.com/fedehub/rt2_assignment1/blob/action/src/state_machine.cpp): By means of an user interface, the user is able of making the robot starts by entering the 1 integer value, the robot starts moving. There is one boolean value which becomes true and then
   * call the position_service.cpp which retrieves the random goal position to reach from the RandomPosition.srv custom service
   * sends the random position as the action server goal
   * waits for the robot to reach the designated position 

### URDF folder 
This folder contains the robot description.
[my_robot.urdf](https://github.com/fedehub/rt2_assignment1/blob/action/urdf/my_robot.urdf)

## Rqt-graph (ROS tool)

By running the following command:

```
rosrun rqt_graph rqt_graph

```
it is possible to show a dynamic graph, depicting what is going on within the System.

![rqt_graph](https://github.com/fedehub/rt2_assignment1/blob/action/rqt-graph/rosgraph.png "Rqt_graph second assignment")

## Documentation

The documentation of this project, obtained by means of **DoxyGen** is visible, within the [docs](https://github.com/fedehub/rt2_assignment1/tree/action/docs) folder

## How to run the code 
The package contains the nodes and the simulation environment for controlling a mobile robot in the Gazebo simulation environment.
To launch the node, please run:
```
roslaunch rt2_assignment1 sim.launch

```

## Release History

* `0.1.0`
 * The first proper release

* `0.0.1`
 * Work in progress

## Meta

Federico Civetta– s4194543 – fedeunivers@gmail.com


https://github.com/fedehub

## Contributing

1. Fork it (https://github.com/fedehub/rt2_assignment1/fork)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request
