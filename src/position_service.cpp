/**
* @file position_service.cpp
* @brief Node implementing the ROS service for gettinf random position
* @author Federico Civetta
* @version 0.1
* @date 13/06/2021
*
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
*  /position_server
*
* Description:
*
* This node advertises a position service. When the service is require, a request
* containing the minimum and the maximum values for the x and y position is used
* to generate a random position between x (or y) min and x (or y) max.
*
*/


#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
 * @brief Random number generator 
 * @param M  defines the minimum possible value for a random number (within a certain interval)
 * @param N the maximum possible value for a random number (within a certain interval)
 
 * @retval a double value, defining a random number adressed to myrandom
 *
 * This function generates a random number between M and N 
 */

double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/**
 * @brief This function is the callback of the service server 
 * @param req  the request received from the client. It gets two intervals of values for x and y 
 * @param res  the response returned to the client (the random coordinates within a specific interval)
 * 
 * @retval the boolean 
 * 
 * 
 */
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}


int main(int argc, char **argv)
{
   /* initialising the random_position_server node */
   ros::init(argc, argv, "random_position_server");
   /* setting-up the node handle n*/
   ros::NodeHandle n;
   /* defining the service and specifying the callback function myrandom*/
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}





