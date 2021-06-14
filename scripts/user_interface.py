#! /usr/bin/env python
"""
.. module:: user_interface
    :platform: Unix
    :synopsis: Python module for piloting the robot to the target

.. moduleauthor:: Federico fedeunivers@gmail.com

ROS node for driving a robot to a specific point

Subscribes to:
/odom topic where the simulator publishes the robot position

Publishes to:
/cmd_vel the desired robot position

Service :
/go_to_point to start the robot motion.

"""
import rospy
import time
from rt2_assignment1.srv import Command
import actionlib
import rt2_assignment1.msg
from geometry_msgs.msg import Twist

## Documentation for the main function.
#
#  More details.
#
#
# @var ui_client defines a service client of user_interface type. It taskes as argument the Command service to activate/deactivate robot behaviours according to user's preferences  
# @var client defines the action client  of the go_to_point Action. As argument it takes the message of type GoalReaching action belonging to the rt2assignment1 package
# @var pub It defines the publsiher of the cmd_vel toic. It publishes a twist message to stop the robot. 
# @var rate useful to fix the frequency of the loop
# @var x it stores the input value when the user interface gets prompted to the user 

def main():
    # Initialising the user_interface node
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    client = actionlib.SimpleActionClient('go_to_point', rt2_assignment1.msg.GoalReachingAction)
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        # If entered value is equal to 1 
        if (x == 1):
            # a request is sent to the go_to_point server to activate the robot
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))            
            
        else:
        # if entered value is equal to zero, then he robot's action goals are canceled     
            client.cancel_all_goals()
            # i decleare a msg of type twist to stop the robot
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.linear.y=0
            twist_msg.angular.z = 0
            # i publish the message
            pub_.publish(twist_msg)
            # the ui_client ' argument is settled to stp in order to deactivate the behaviour 
            ui_client("stop")
            # asking for another input
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()


