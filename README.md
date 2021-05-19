# First Assignment of the Research Track 2 course - Ros2 branch 

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

1. a launch file to start the container manager and the components that have been created
2. a script to launch all required nodes and the simulation have been implemented

### Launch folder
It contains two different files:
1. [all_launch.py](https://github.com/fedehub/rt2_assignment1/blob/ros2/launch/all_launch.py): this script allow us to launch the **container manager** where all the *composable nodes* are contained
2. [sim2.launch](https://github.com/fedehub/rt2_assignment1/blob/ros2/launch/sim2.launch): This launch file permits to launch the overall simulation (by also involving the `goToPoint` and the `userInterface` nodes)

### Srv folder
It contains three different files:
1. [Command.srv]()
2. [Position.srv]()
3. [RandomPosition.srv]()
### Src folder 
As already mentioned a few paragraphs above, within this folder, it is possible to see the implementation of:
* [position_service.cpp](https://github.com/fedehub/rt2_assignment1/blob/ros2/src/position_service.cpp)
* [state_machine.cpp](https://github.com/fedehub/rt2_assignment1/blob/ros2/src/state_machine.cpp)
The beahviour of these two nodes is the same of the main branch ones, with the unique exeption that these are compatble with ROS2. 

## How to run the code 
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