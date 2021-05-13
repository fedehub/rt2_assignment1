#include <inttypes.h>
#include <memory>
#include <functional>
#include "rt2_assignment1/srv/random_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"


using std::placeholders::_1; // they will be replaced by the actual message during the run
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{
class Position_Service : public rclcpp::Node
{
public:
  Position_Service(const rclcpp::NodeOptions & options)
  : Node("position_server", options)
  {
    service_ = this->create_service<rt2_assignment1::srv::RandomPosition>(
      "/position_server", std::bind(&Position_Service::handle_service, this, _1, _2, _3)); // callback
  }

private:
  
  void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<rt2_assignment1::srv::RandomPosition::Request> request,
  const std::shared_ptr<rt2_assignment1::srv::RandomPosition::Response> response)
  {
  (void)request_header;
    std::cout<<"received request"<<std::endl;
    response->x = request->x_min + (rand() / ( RAND_MAX / (request->x_max-request->x_min) ) );
    response->y = request->y_min + (rand() / ( RAND_MAX / (request->y_max-request->y_min) ) );
    response->theta =  -3.14+ (rand() / ( RAND_MAX / (6.28)));
    std::cout<<"sent random"<<std::endl;
  }
  rclcpp::Service<rt2_assignment1::srv::RandomPosition>::SharedPtr service_; // service var: pointer to rclcpp service
  
  
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::Position_Service)
