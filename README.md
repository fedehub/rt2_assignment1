# First assignment _Research Track_ course - second module 

## Table of contents 

-Index
  - [General Instructions](#general-instructions)
  - [Vrep scene](#vrep-scene)
  - [Foundamental Packages](#foundamental-packages)
  - [The "*action*" branch](#the-action-branch)
  - [The "*ros2*" branch](#the-ros2-branch)
  - [Scripts folder](#scripts-folder)
  - [SRV folder](#srv-folder)
  - [Launch folder](#launch-folder)
  - [Launch the simulation](#launch-the-simulation)
  - [Release History](#release-history)
  - [Meta](#meta)
  - [Contributing](#contributing)
  
## General Instructions

The [Professor's package](https://github.com/CarmineD8/rt2_assignment1) has been modified by creating this repository, containing the given package and two additional branches:

1. `action` 
2. `ros2`

Finally, in the main branch, I added a Vrep scene containing the robot interacting with the simulation. I will briefly introduce it in the following section

## Vrep scene 
The [sceneVrep.ttt](https://github.com/fedehub/rt2_assignment1/blob/main/sceneVrep.ttt) file contains an *ad-hoc scene* adressed to the robot simulator (**Coppelliasim**). Moreover, a `robot_pioneer_ctrl` has been placed within the Coppelliasim environment. The robot's motion is controlled through a ROS node belonging to the rt2_assignment package, inside the main branch. 
In order to communicate with the robot, a pubisher to the  `/odom`topic has been decleared (within the robot child script). It is responsible for the announcement of the actual robot postion which will be then exploited by the subsciber of the `/odom` topic ***go_to_point.py***. To conclude with, to set the actuator velocity a subscriber to the `/cmd_vel` topic has been introduced by means Twist message published within the  ***go_to_point.py*** script.

## Foundamental Packages
* [SimExtRos](https://github.com/CoppeliaRobotics/simExtROS2): The ROS2 Interface plugin for CoppeliaSim
* [Coppeliasim]() it allows to integrate Vrep with ROS 
* [Ros1_bridge](https://github.com/ros2/ros1_bridge) to let some portions of code written in ROS communicate with code in ROS2


## The "*action*" branch
This latter, should contain the same package in ROS, but with the **go_to_point** node modelled as a ROS action server, instead of a “simple” server.
Given that, **the robot FSM node** should now implement mechanisms for possibly cancelling the goal, when the related user command is received. For more details, please click [here](https://github.com/fedehub/rt2_assignment1/tree/action)

## The "*ros2*" branch
In the branch `ros2`, the cpp nodes (Robot FSM and position server) should be written for ROS2, as **components**, so that, by using the _**ros1_bridge**_, they can be interface with the ROS nodes and with the simulation in Gazebo. **The go_to_point** can still be implemented as a service.
Also:

1. a _**launch file**_ to start the container manager and the components, should be created
2. a _**script**_ to launch all required nodes and the simulation should be implemented

For more details, please click [here](https://github.com/fedehub/rt2_assignment1/tree/ros2)

## Scripts Folder
Here below is reported a brief description of the content of this folder:
* [user_interface.py]() implementing the user interface which basically allows the user to:
  * start the robot movement  by pressing `1`
  * stopping the robot movement by pressing `0`
* [go_to_point.py]() implementing a service to drive a robot toward a point in the environment  



## Src Folder
Here we can find, as for the `ros2` branch, two files, respectively:
1. [state_machine.cpp]() The SM has been implemented as a *service server* whose activation depends on a boolean variable (*start*) retrieved by the `ui_client`, as request, within the [user_interface.py]. The [state_machine.cpp]() node also constitutes a *service client* for both:
   * `/go_to_point`
   * `/position_server`
 
>REMARK: if the boolean varaiable "start" is set to true, it asks for a random position to then set its response equal to the request values of the the `/go_to_point` service.
 
1. [position_service.cpp]() It is implemented as a *service Server* Node and it replies with random values for x,y and theta where x and y should be limited between some max and min values (given as request)

## SRV folder
It basically contains three different custom services:
1. [Command](https://github.com/fedehub/rt2_assignment1/blob/main/srv/Command.srv): It takes:
   * as request: a string
   * as reply:  a Boolean 
2. [RandomPosition]() It takes:
   * as request: a minimum and a maximum x-y coordinates' value
   * as reply: a random x, y and theta angle value 
3. [Position]() It takes:
   * as request: a position to reach 
   * as reply: a boolean  

## Launch folder
Here we can find the three launch files, singularly exploited among the branches
1. [simVrep.launch]() Launch file for starting the four nodes interacting with Vrep
2. [sim.launch]() Launch file for starting the four nodes and the overall simulation
3. [sim2.launch]() Launch file for starting the container manager and the components which have been created 


## Launch the simulation 

To launch the node, please 
1. download coppelliasim at: http://www.coppeliarobotics.com/downloads.html
2. launch the ROS master in background 
  ```
roscore &

  ```
3. In a second tab, please run  
  ```
./coppeliasim.sh

  ```
4. Load the scene in Coppeliasim 
5. Open a third terminal tab and launc
 ```
roslaunch rt2_assignment1 simVrep.launch

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
