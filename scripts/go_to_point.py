#! /usr/bin/env python

"""
.. module:: go_to_point
    :platform: Unix
    :synopsis: Python module for piloting the robot to the target

.. moduleauthor:: Federico fedeunivers@gmail.com

ROS node for driving a robot to a specific point within a simulated
environment, given a certain orientation. To start with, the robot tries
to orient itself accordingly to the target goal.

Subscribes to:
/odom topic where the simulator publishes the robot position

Publishes to:
/cmd_vel the desired robot position

Service :
/go_to_point to start the robot motion.

"""


import rt2_assignment1.msg
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from rt2_assignment1.srv import Position
import math
import actionlib


# robot state variables
position_ = Point()
"""Point: current robot position
"""
yaw_ = 0
"""Pose: current robot orientation
"""
position_ = 0
"""Float: current robot angle
"""
state_ = 0
"""Int: current state of the server
"""

pub_ = None
"""None: It publish a twist message on the */cmd_vel* topic
"""

# parameters for control
yaw_precision_ = math.pi / 9
"""Float: yaw acc +/- 20 deg allowed
"""
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
"""Float: tight yaw acc +/- 2
"""
dist_precision_ = 0.1
"""Float: linear distance allowed
"""
kp_a = -3.0
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6


def clbk_odom(msg):
    '''
    Description of the callback:

    This function retrieves the current robot position for saving
    it within the *position_* global variable and is responsible for
    transforming the orientation from quaternion angles to Euler ones

    Args:
      msg(Twist): data retrieved by */cmd_vel* topic

    Returns:
      None

    '''
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


class GoalReachingAction(object):
    """
    This is a the class representing the implementation of the 
    Goal action

    param object: A handle to the :class:`GoalReachingAction` client object 
    :type object: class:`GoalReachingAction`
    :param name: Device MAC address, defaults to None
    :type name:The action name 
    :param name: str
		
    """
    # initialising variables for defining each field of the action
    _feedback = rt2_assignment1.msg.GoalReachingFeedback()
    _result = rt2_assignment1.msg.GoalReachingResult()
    _goal = rt2_assignment1.msg.GoalReachingGoal()



    def __init__(self, name):
        '''Constructor method 
        '''
        self._action_name = name
        # initialisation of the actionlib server.As arguments it gets the name
        # of the action and the msg of type GoalREaching action and the
        # callback execute_cb.
        self._as = actionlib.SimpleActionServer(
    self._action_name,
    rt2_assignment1.msg.GoalReachingAction,
    execute_cb=self.execute_cb,
     auto_start=False)
        # starting the action server
        self._as.start()



    def execute_cb(self, goal):
        '''
        Description of the callback:

        This function takes as argument the goal variable whose value is
        provided from the action client in the state_machine.cpp. The feedback
        of the action message is constantly updated as the current pose of
        the robot. Then the desired_position is initialised as the goal of
        the action. To conclude with, the change_state function (which is responsible
        for the _state assignment) is given with argument zero so that the robot
        can start fixing its own yaw before proceeding by reaching the goal

        Args:
          goal(Float): the robot postion goal

        Returns:
		  goal(float)

        '''
        global position_, yaw_precision_, yaw_, state_, pub_
        # helper variables
        r = rospy.Rate(1)
        # boolean variable initialisation
        success = True

        # initialising feedback fields
        self._feedback.updated_x = position_.x
        self._feedback.updated_y = position_.y
        self._feedback.updated_theta = yaw_

        # start executing the action
        desired_position = Point()
        desired_position.x = goal.x
        desired_position.y = goal.y
        des_yaw = goal.theta
        self.change_state(0)

        while True:
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            # updating feedback (run-time)
            self._feedback.updated_x = position_.x
            self._feedback.updated_y = position_.y
            self._feedback.updated_theta = yaw_
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            if state_ == 0:
                self.fix_yaw(desired_position)
            elif state_ == 1:
                self.go_straight_ahead(desired_position)
            elif state_ == 2:
                self.fix_final_yaw(des_yaw)
            elif state_ == 3:
                self.done()
                break



    def change_state(self,state):
        '''
        Description of the change_state function:

        This value retrieve and assigns the current state to the
        global one (*state_*)

        Args:
          state(int): the state of the robot

        Returns:
          None

        '''
        global state_
        state_ = state
        print('State changed to [%s]' % state_)

    def normalize_angle(self,angle):
        '''
        Function for normalizing the angle between -pi and pi.
        
        Args:
          angle(Float): the input angle
        
        Returns:
          angle(Float): the normalized angle.

        '''
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def fix_yaw(self,des_pos):
        '''
        Description of the fix_yaw function:

		This function computes the robot orientation among x and y 
		coordinates and sets the angular velocity needed for achieving
		the desired robot position. 
		   
		Args:
		  des_pos(Point):  the expected x and y coordinates

		Returns:
		   None

        '''
        desired_yaw = math.atan2(
            des_pos.y - position_.y,
            des_pos.x - position_.x)
        err_yaw = self.normalize_angle(desired_yaw - yaw_)
        rospy.loginfo(err_yaw)
        twist_msg = Twist()
        if math.fabs(err_yaw) > yaw_precision_2_:
            twist_msg.angular.z = kp_a * err_yaw
            if twist_msg.angular.z > ub_a:
                twist_msg.angular.z = ub_a
            elif twist_msg.angular.z < lb_a:
                twist_msg.angular.z = lb_a
        pub_.publish(twist_msg)
        # state change conditions
        if math.fabs(err_yaw) <= yaw_precision_2_:
            # print ('Yaw error: [%s]' % err_yaw)
            self.change_state(1)

    def go_straight_ahead(self,des_pos):
        '''
		Description of the go_straight_ahead function:

		This function computes the robot orientation among x and y 
		coordinates necessary to reach the x,y target point. Once the
		linear velocities have been set, an angular velocity is defined
		by means of an error. It is proportional to this latter and it 
		allows a correction of the trajectory, by checking a treshold
		over a distance
		   
		   
		Args:
		  des_pos(Point): the expected x and y coordinates

		Returns:
		  None

        '''
        desired_yaw = math.atan2(
            des_pos.y - position_.y,
            des_pos.x - position_.x)
        err_yaw = desired_yaw - yaw_
        err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                            pow(des_pos.x - position_.x, 2))
        err_yaw = self.normalize_angle(desired_yaw - yaw_)
        rospy.loginfo(err_yaw)
        if err_pos > dist_precision_:
            twist_msg = Twist()
            twist_msg.linear.x = 0.3
            if twist_msg.linear.x > ub_d:
                twist_msg.linear.x = ub_d
            twist_msg.angular.z = kp_a * err_yaw
            pub_.publish(twist_msg)
        else:  # state change conditions
        # print ('Position error: [%s]' % err_pos)
            self.change_state(2)
        # state change conditions
        if math.fabs(err_yaw) > yaw_precision_:
            # print ('Yaw error: [%s]' % err_yaw)
            self.change_state(0)

    def fix_final_yaw(self,des_yaw):
        '''
        Description of the fix_final_yaw function:

        This function computes the error between the desired robot
		orientation and the current one.

		Args:
		  des_yaw(Float): expected orientation

		Returns:
		  None

        '''
        err_yaw = self.normalize_angle(des_yaw - yaw_)
        rospy.loginfo(err_yaw)
        twist_msg = Twist()
        if math.fabs(err_yaw) > yaw_precision_2_:
            twist_msg.angular.z = kp_a * err_yaw
            if twist_msg.angular.z > ub_a:
                twist_msg.angular.z = ub_a
            elif twist_msg.angular.z < lb_a:
                twist_msg.angular.z = lb_a
        pub_.publish(twist_msg)
        # state change conditions
        if math.fabs(err_yaw) <= yaw_precision_2_:
            # print ('Yaw error: [%s]' % err_yaw)
            self.change_state(3)

    def done(self):
        """
		Description of done function:
		    
        This function marks the goal target as succeeded, once all the
        linear and angular velocities are set to zero  
    
        Args :
          None
    
        Returns :
          None
          
        """
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        pub_.publish(twist_msg)
        self._result.ok = True
        rospy.loginfo(' Succeeded in reaching the desired Position! ')
        self._as.set_succeeded(self._result)



def main():
    """
    Description of the main function:
           
    As the go_to_point node is called it runs. Here the node
    gets initialized and the server needed for the :class:`GoalReachingAction`
    is declared.
    Moreover, a global variable is needed (*pub_*) for defining
    a publisher to the */cmd_vel* topic.ß
           
    
    Args :
      None
    
    Returns :
      None
             
    """
    global pub_
    rospy.init_node('go_to_point')
    server = GoalReachingAction('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    rospy.spin()


if __name__ == '__main__':
    main()
