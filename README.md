# First Assignment of the Research Track 2 course - Ros2 branch 

## Table of Contents
- Index
  - [List of the required packages](#list-of-the-required-packages)
  - [Description of the branch](#description-of-the-branch)
    - [Launch folder](#launch-folder)
    - [Srv folder](#srv-folder)
    - [Src folder](#src-folder)
    - [mapping rule](#mapping-rule)
  - [Rqt-graph (ROS tool)](#rqt-graph-ros-tool)
  - [How to launch](#how-to-launch)
  - [Documentation](#documentation)
  - [Release History](#release-history)
  - [Meta](#meta)
  - [Contributing](#contributing)

## List of the required packages 
The packages required to build the Node, are:
1. `rt2_assignment1` [main branch](https://github.com/fedehub/rt2_assignment1/tree/main) on a ros workspace (in the `/root` folder, source ros1.sh)
2. `rt2_assignment1` [ros2 branch](https://github.com/fedehub/rt2_assignment1/tree/ros2) on a ros2 workspace (in the `/root` folder, source ros2.sh)
3. `ROS1_bridge` [more here](https://github.com/ros2/ros1_bridge) on a ros12 workspace (in the `/root` folder, source ros12.sh)

## Description of the branch 
In the `ros2` [branch](https://github.com/fedehub/rt2_assignment1/tree/ros2), the cpp nodes:
*  [state_machine.cpp](https://github.com/fedehub/rt2_assignment1/blob/ros2/src/state_machine.cpp): it contains the Robot FSM
*  [position_service.cpp](https://github.com/fedehub/rt2_assignment1/blob/ros2/src/position_service.cpp): it contains the Position server 
They have been designed for being compatible with `ROS2`, as **components**, so that, by using the `ros1_bridge`, they can be interface with both:
* the ROS nodes 
* the simulation in Gazebo. 
The `go_to_point` has been implemented as a service. Also:

1. a **launch file** to start the container manager and the components, has been created
2. a **script** to launch all required nodes and the simulation, has been implemented

### Launch folder
It contains a script file:
1. [all_launch.py](https://github.com/fedehub/rt2_assignment1/blob/ros2/launch/all_launch.py): this script allow us to launch the **container manager** where all the *composable nodes* are contained


### Srv folder
It contains three different files:
1. [Command.srv](https://github.com/fedehub/rt2_assignment1/blob/ros2/srv/Command.srv)
2. [Position.srv](https://github.com/fedehub/rt2_assignment1/blob/ros2/srv/Position.srv)
3. [RandomPosition.srv](https://github.com/fedehub/rt2_assignment1/blob/ros2/srv/RandomPosition.srv)
### Src folder 
As already mentioned a few paragraphs above, within this folder, it is possible to see the implementation of:
* [position_service.cpp](https://github.com/fedehub/rt2_assignment1/blob/ros2/src/position_service.cpp) It is implemented as a *service Server* Node and it replies with random values for x,y and theta where x and y should be limited between some max and min values (given as request)
* [state_machine.cpp](https://github.com/fedehub/rt2_assignment1/blob/ros2/src/state_machine.cpp)
The beahviour of these two nodes is the same of the main branch ones, with the unique exeption that these are compatble with ROS2. 

## Mapping Rule

We need to create a [mapping_rules.yaml]() file in the ROS2 package for compiling the bridge 

## Rqt-graph (ROS tool)

By running the following command:

```
rosrun rqt_graph rqt_graph

```
it is possible to show a dynamic graph, depicting what is going on within the System.

![rqt_graph]( https://github.com/fedehub/rt2_assignment1/blob/ros2/rqt-graph/rosgraph.png "Rqt_graph first graph")
and
![rqt_graph]( https://github.com/fedehub/rt2_assignment1/blob/ros2/rqt-graph/rosgraphros.png "Rqt_graph second graph")

## How to launch 
In order to run the code, please follow the steps reported below:
1. Create three `.sh` files in your `/root` directory: **the first one** (ros.sh) should contain these lines:
   
   ```
   #!/bin/bash
   source /root/my_ros2/install/setup.bash
   ```
   The **second one** (ros2.sh) should contain:

    ```
    #!/bin/bash
    source /root/my_ros2/install/setup.bash
    ```
   The **third one** (ros12.sh) should contain:
    ```
    #!/bin/bash
    source /root/my_ros/devel/setup.bash
    source /opt/ros/foxy/setup.bash
    source /root/my_ros2/install/local_setup.bash
    ```
2. Open a new shell and run:
    
   ```
   source ros.sh
   ```
3. Then, to run the simulation, please enter in the same shell:  
   
   ```
   roslaunch rt2_assignment1 sim2.launch
   ```
4. To teest the bridge, open another shell and run
  
   ```
   source ros12.sh
   ```
5. Then, enter:

   ```
     ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
     
   ```
6. To conclude with, open a third shell and run
    
   ```
   source ros2.sh
   ```
7. Hence, to launch the overall simulation, enter
   
   ```
     ros2 launch rt2_assignment1 all_launch.py
     
   ```
Otherwise:

 1. Go inside the `\root` folder (or wherever all the files (ros.sh,ros2.sh,ros12.sh, bridge) are contained
 2. Open a shell, give to the **all_simulation.she** file  all the necessary permissions and run
       ```
      ./all_simulation.sh

       ```

## Documentation

The documentation of this project, obtained by means of **DoxyGen** is visible, within the [docs](https://github.com/fedehub/rt2_assignment1/tree/ros2/docs) folder

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