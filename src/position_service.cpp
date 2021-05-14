#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
 *@brief This returns a random value
 *@param M  the minimum of a certain interval
 *@param N the maximum of a certain interval 
 *@retval a double value, defining a random number adressed to myrandom
 */

double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/**
 *@brief This function is the callback of the service server 
 *@param req  the request received from the client. It gets two intervals of values for x and y 
 *@param res  the response returned to the client (the random coordinates within a specific interval)
 *@retval the boolean 
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





