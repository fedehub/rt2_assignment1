# First assignment _Research Track_ course - second module 

## Table of contents 

-Index
  - [General Instructions](#general-instructions)
  - [Vrep scene](#vrep-scene)
  - [The "*action*" branch](#the-action-branch)
  - [The "*ros2*" branch](#the-ros2-branch)
  - [Launch the node](#launch-the-node)
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



## The "*action*" branch
This latter, should contain the same package in ROS, but with the **go_to_point** node modelled as a ROS action server, instead of a “simple” server.
Given that, **the robot FSM node** should now implement mechanisms for possibly cancelling the goal, when the related user command is received. For more details, please click [here](https://github.com/fedehub/rt2_assignment1/tree/action)

## The "*ros2*" branch
In the branch `ros2`, the cpp nodes (Robot FSM and position server) should be written for ROS2, as **components**, so that, by using the _**ros1_bridge**_, they can be interface with the ROS nodes and with the simulation in Gazebo. **The go_to_point** can still be implemented as a service.
Also:

1. a _**launch file**_ to start the container manager and the components, should be created
2. a _**script**_ to launch all required nodes and the simulation should be implemented

For more details, please click [here](https://github.com/fedehub/rt2_assignment1/tree/ros2)



## Launch the node 

To launch the node, please run:
```
rosrun rt2_assignment1 sim.launch

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
