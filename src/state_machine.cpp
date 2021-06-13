/**
* @file state_machine.cpp
* @brief Node describing the state machine implementation 
* @author Federico Civetta
* @version 0.0.1
* @date 13/06/2021
*
* @param [in] world_width Definte the width of the discretized world
*
* @details 
*
* Subscribes to: <BR>
*  /robot_behavior_state
* Publishes to: <BR>
*  /PlayWithRobot
*
* Services: <BR>
*  /user_interface
*  /position_server
*
* Description:
*
* By means of an user interface, the user is able of making the robot starts 
* by entering the 1 integer value, the robot starts moving. There is one boolean 
* value which becomes true and then call the \verbatim position_service.cpp  \endverbatim
* which retrieves the random goal position to reach from the RandomPosition.srv custom 
* service, sends the random position as the action server goal, waits for the robot 
* to reach the designated position
*
*/


#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rt2_assignment1/GoalReachingAction.h>

bool start = false;

/**
 *@brief This function is the callback function of the service for server.
 *@param req  the request received from the client of the user_interface.py. 
 *@param res  the response has not been used 
 *@retval A boolean value
 */

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
   /* if the user has entered 1, then the request of the command custom service  is a string, 
      initialised as "start" */
    if (req.command == "start"){
      /* the global boolean start is set to True*/
    	start = true;
    }
    /* else if the user has entered 0*/  
    else {
      /* the global boolean start is set to False*/ 
    	start = false;
    }
    return true;
}


int main(int argc, char **argv)
{
   /* Initialising the state_machine node*/
   ros::init(argc, argv, "state_machine");
   /* setting-up the node handler n*/
   ros::NodeHandle n;
   /* initialising the /user_interface service */
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   /* initialising the client for retreving the random position by means of the /position_server service */
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   /* creating the action client */
   /* true causes the client to spin its own thread */
   actionlib::SimpleActionClient<rt2_assignment1::GoalReachingAction> ac("go_to_point", true);
   /* initialising a custom message of type RandomPosition */
   rt2_assignment1::RandomPosition rp;
   /* initialising a custom  message of typer GoarReaching goal */
   rt2_assignment1::GoalReachingGoal goal;
   
   /* filling the custom message request fields */
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
 
   
   while(ros::ok()){
   	ros::spinOnce();
      /* if star var is True*/ 
   	if (start){
         /* call for the Service random position */
   		client_rp.call(rp);
   		ROS_INFO("Waiting for action server to start.");
  		/* wait for the action server to start*/
  		ac.waitForServer(); // will wait for infinite time
      /* initialising goal's fields with retrieved random values */
  		goal.x = rp.response.x;
  		goal.y = rp.response.y;
  		goal.theta = rp.response.theta;
  		std::cout << "\nGoing to the position: x= " << goal.x << " y= " << goal.y << " theta = " <<goal.theta << std::endl;
  		/* sending the goal to the action server */
      ac.sendGoal(goal);
  		
		/* wait for the action to return until the robot reach the desired postion */
 		bool finished_before_timeout = ac.waitForResult(ros::Duration(120.0));
		
      
		if (finished_before_timeout)
		{
		   ROS_INFO("Postion reacheded ");
		}
		else
		   ROS_INFO("Action did not finish before the time out.");
   		
   	}
   }
   return 0;
}





