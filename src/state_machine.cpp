
#include <memory>
#include <chrono>
#include <cinttypes>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "rt2_assignment1/srv/random_position.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/command.hpp"
#include "rclcpp_components/register_node_macro.hpp"
//position tipo di servizio
//canale run time


using std::placeholders::_1; // they will be replaced by the actual message during the run
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{
class FSM : public rclcpp::Node
{
public:
  FSM(const rclcpp::NodeOptions & options)
  : Node("state_machine", options)
  {
    service_ = this->create_service<rt2_assignment1::srv::Command>(
      "/user_interface", std::bind(&FSM::handle_service, this, _1, _2, _3)); // callback
    client_1 = this->create_client<rt2_assignment1::srv::Position>("/go_to_point");
    while (!client_1->wait_for_service(std::chrono::seconds(1))){
     if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client_1 interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");}
    client_2 = this->create_client<rt2_assignment1::srv::RandomPosition>("/position_server"); 
    while (!client_2->wait_for_service(std::chrono::seconds(1))){
     if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client_2 interrupted while waiting for service to appear.");
      return;
    }
     RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }

    //this->state_mach();
  }

  

private:
  bool  start = false;
  void state_mach(){
            if (this->start){
	   std::cout<<"inside the state machine"<<std::endl;
	   auto request_1 = std::make_shared<rt2_assignment1::srv::Position::Request>();
	  // auto response_1 = std::make_shared<rt2_assignment1::srv::Position::Response>();
	   auto request_2 = std::make_shared<rt2_assignment1::srv::RandomPosition::Request>();
	   //auto response_2 = std::make_shared<rt2_assignment1::srv::RandomPosition::Response>();
	   request_2->x_max = 5.0;
	   request_2->x_min = -5.0;
	   request_2->y_max = 5.0;
	   request_2->y_min = -5.0;
	   std::cout<<"write rndm msg"<<std::endl;
	  // using ServiceResponseFuture =
	   //rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedFuture;
	   auto response_received_callback = [this](rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedFuture future) {
	              std::cout<<"received random response"<<std::endl;
		      this->response_2=future.get();
		      //this->response_2->theta=future.get()->theta;
		      //this->response_2->x=future.get()->x;
    	   };
           auto future_result = client_2->async_send_request(request_2, response_received_callback);
	   
	  
	   	
	   
	   		
	   		request_1->x = this->response_2->x;
	   		request_1->y = this->response_2->y;
	   		request_1->theta = this->response_2->theta;
	   		std::cout << "\nGoing to the position: x= " << request_1->x << " y= " <<request_1->y << " theta = " <<request_1->theta << std::endl;
	   		auto response_received_callback2 = [this](rclcpp::Client<rt2_assignment1::srv::Position>::SharedFuture future) {
	                     (void)future;
	                    std::cout << "Position reached" << std::endl;
		     
    	               };
	   		auto result_1 = client_1->async_send_request(request_1, response_received_callback2);
	   		this->state_mach();
	   	}
	  
  }

  void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<rt2_assignment1::srv::Command::Request> request,
  const std::shared_ptr<rt2_assignment1::srv::Command::Response> response)
  {
  (void)request_header;
  //(void)response;
     if (request->command == "start"){
        std::cout<<"start is true"<<std::endl;
    	this->start = true;
    	response->ok=true;
    	this->state_mach();
    }
    else {
    	this->start = false;
    	response->ok=true;
    	std::cout<<"close"<<std::endl;
    }
   // response->ok=true;
    }
    
  rclcpp::Service<rt2_assignment1::srv::Command>::SharedPtr service_; // service var: pointer to rclcpp service
  rclcpp::Client<rt2_assignment1::srv::Position>::SharedPtr client_1; // service var: pointer to rclcpp service
  rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedPtr client_2; // service var: pointer to rclcpp service
  std::shared_ptr<rt2_assignment1::srv::RandomPosition::Response> response_2;
  
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::FSM)
