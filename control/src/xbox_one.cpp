/**
 * @xbox UAV control 
 * @file xbox_one.cpp
 * @author Tiger Wuu <tiger871108@gmail.com>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>

#include <sensor_msgs/msg/joy.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> //WTF?
#include <std_msgs/msg/float32_multi_array.hpp>

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <cmath>
#include <iomanip>

#include <eigen3/Eigen/Dense>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace Eigen;
using std::placeholders::_1;

class ManualControl : public rclcpp::Node
{
public:
	ManualControl() : Node("manual_control")
	{
		// publisher
		manual_cmd_publisher_ = this->create_publisher<ManualControlSetpoint>("/fmu/in/manual_control_input", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		// subscriber
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", qos, std::bind(&ManualControl::joy_callback,this, _1));		
		// client
		vehicle_command_client_ = this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");
		
		while (!vehicle_command_client_->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
		}
		manual_cmd.valid = true;
		manual_cmd.data_source = 2;
		// manual_cmd.sticks_moving = true;
	
		manual_cmd.roll = 0;
    	manual_cmd.pitch = 0;
    	manual_cmd.yaw = 0;
    	manual_cmd.throttle = 0;    
		/* auto timer_callback = [this]() -> void {
			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			}
			// calculate the formation geometry
			this->formation_geometry();
			// offboard_control_mode needs to be paired with trajectory_setpoint
			this->publish_offboard_control_mode(attitude = true);
			this->Lyapunov_based_formation_controller()
			//publish_trajectory_setpoint();

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};*/
		
		this->timer_ = this->create_wall_timer(100ms, std::bind(&ManualControl::manual_cmd_publish, this));
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<ManualControlSetpoint>::SharedPtr manual_cmd_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_ ;
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	
	// service
	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
	// service states
	bool service_done_= false;
	uint8_t service_result_;

	ManualControlSetpoint manual_cmd{};
	
	void request_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);
	void manual_cmd_publish();
	
	void joy_wire(const sensor_msgs::msg::Joy &msg);
	void joy_bluetooth(const sensor_msgs::msg::Joy &msg);
	void joy_callback(const sensor_msgs::msg::Joy &msg);
	void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
	
	// joy
	float LRleft,UDleft,LT,LRright,UDright,RT,ckLR,ckUD;
	int A,B,X,Y,LB,RB,back,start,power,BSL,BSR;
	// mode 
	int armed = 0;
	int disarmed = 0;
	
	int manual = 0;
	int position = 0;
	int land = 0;
	int takeoff = 0;
	int hold = 0;
	int mission = 0;
	int offboard = 0;
	int rtl = 0;

	float transition = 0.0;
	
};
// UniquePtr instead?
void ManualControl::joy_wire(const sensor_msgs::msg::Joy &msg)
{
	LRleft = msg.axes[0];		
	UDleft = msg.axes[1];		
	LT = msg.axes[2];		
	LRright = msg.axes[3];		
	UDright = msg.axes[4];		
	RT = msg.axes[5];		
	ckLR = msg.axes[6];		
	ckUD = msg.axes[7];	

	A = msg.buttons[0];
	B = msg.buttons[1];
	X = msg.buttons[2];
	Y = msg.buttons[3];
	LB = msg.buttons[4];
	RB = msg.buttons[5];
	back = msg.buttons[6];
	start = msg.buttons[7];
	power = msg.buttons[8];
	BSL = msg.buttons[9];
	BSR = msg.buttons[10];

}
void ManualControl::joy_bluetooth(const sensor_msgs::msg::Joy &msg)
{
	LRleft = msg.axes[0];		
	UDleft = msg.axes[1];		
	LT = msg.axes[5];		
	LRright = msg.axes[2];		
	UDright = msg.axes[3];		
	RT = msg.axes[4];		
	ckLR = msg.axes[6];		
	ckUD = msg.axes[7];	

	A = msg.buttons[0];
	B = msg.buttons[1];
	X = msg.buttons[3];
	Y = msg.buttons[4];
	LB = msg.buttons[6];
	RB = msg.buttons[7];
	back = msg.buttons[10];
	start = msg.buttons[11];
	power = msg.buttons[12];
	BSL = msg.buttons[13];
	BSR = msg.buttons[14];

}

void ManualControl::joy_callback(const sensor_msgs::msg::Joy &msg)
{	
	joy_wire(msg);
	// joy_bluetooth(msg);
    
	// UDleft = (UDleft+1)/2;
	manual_cmd.roll = -LRright;
    manual_cmd.pitch = UDright;
    manual_cmd.yaw = -LRleft;
    manual_cmd.throttle = UDleft;    
	manual_cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	manual_cmd.timestamp_sample = this->get_clock()->now().nanoseconds() / 1000;
	armed = LT;
    disarmed = RT;

    manual = LB;
    position = RB;
    land = ckUD;
    takeoff = ckUD;
    hold = Y;
    mission = A;
    offboard = B;
    rtl = X;
	
	transition = ckLR;
	// this->manual_cmd_publish();
}

void ManualControl::manual_cmd_publish()
{	
	if (armed == -1.0){
	    this->request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	}
	if (disarmed == -1.0){
	    this->request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	}
	if (manual == 1){
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
	}
	else if (position == 1){
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3);
	}
	else if (land == -1){
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
	}
	else if (takeoff == 1){
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF);
	}
	else if (hold == 1){
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 3);
	}
	else if (mission == 1){
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 4);
	}
	else if (offboard == 1){
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	}	
	else if (rtl == 1){
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 5);
	}
	if (transition == 1){
	    // MR
		this->request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, 3, 1);		
	}
	else if (transition == -1){
	    // FW
		this->request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, 4, 1);		
	}
	
	this->manual_cmd_publisher_->publish(manual_cmd);	
}
void ManualControl::publish_vehicle_command(uint16_t command, float param1 , float param2, float param3, float param4, float param5 , float param6 , float param7)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.param4 = param4;
	msg.param5 = param5;
	msg.param6 = param6;
	msg.param7 = param7;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

void ManualControl::request_vehicle_command(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
	auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.param4 = param4;
	msg.param5 = param5;
	msg.param6 = param6;
	msg.param7 = param7;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	request->request = msg;

	this->service_done_ = false;
	auto result = vehicle_command_client_->async_send_request(request, std::bind(&ManualControl::response_callback, this, std::placeholders::_1));
	RCLCPP_INFO(this->get_logger(), "Command send");
}

void ManualControl::response_callback(
      rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
	  auto reply = future.get()->reply;
	  this->service_result_ = reply.result;
      switch (this->service_result_)
		{
		case reply.VEHICLE_CMD_RESULT_ACCEPTED:
			RCLCPP_INFO(this->get_logger(), "command accepted");
			break;
		case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
			RCLCPP_WARN(this->get_logger(), "command temporarily rejected");
			break;
		case reply.VEHICLE_CMD_RESULT_DENIED:
			RCLCPP_WARN(this->get_logger(), "command denied");
			break;
		case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
			RCLCPP_WARN(this->get_logger(), "command unsupported");
			break;
		case reply.VEHICLE_CMD_RESULT_FAILED:
			RCLCPP_WARN(this->get_logger(), "command failed");
			break;
		case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
			RCLCPP_WARN(this->get_logger(), "command in progress");
			break;
		case reply.VEHICLE_CMD_RESULT_CANCELLED:
			RCLCPP_WARN(this->get_logger(), "command cancelled");
			break;
		default:
			RCLCPP_WARN(this->get_logger(), "command reply unknown");
			break;
		}
      this->service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

int main(int argc, char *argv[])
{
	std::cout << "Manual control(xbox ones) working..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ManualControl>());

	rclcpp::shutdown();
	return 0;
}
