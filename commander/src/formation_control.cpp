/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> //WTF?
#include <std_msgs/msg/float32_multi_array.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <cmath>

#include <eigen3/Eigen/Dense>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace Eigen;
using std::placeholders::_1;

class FormationControl : public rclcpp::Node
{
public:
	FormationControl() : Node("formation_control")
	{
		// publisher
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_attitude_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		
		formation_error_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/data/formation_error", 10);
		// subscriber
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		vehicle_position_subscriber_ = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&FormationControl::vehicle_position_callback,this, _1));
		virtual_leader_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/virtual_leader_information", qos, std::bind(&FormationControl::virtual_leader_callback, this, _1));
		wind_estimation_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/wind_estimation_information", qos, std::bind(&FormationControl::wind_estimation_callback, this, _1));
		vehicle_status_subscriber_ = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status", qos, std::bind(&FormationControl::vehicle_status_callback, this, _1));
		// client
		vehicle_command_client_ = this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");
		// transition form MR to FW
		while (!vehicle_command_client_->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
		}
		this->request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, 4, 1);
	
		auto timer_callback = [this]() -> void {
			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			}
			// calculate the formation geometry
			this->formation_geometry();
			// offboard_control_mode needs to be paired with trajectory_setpoint
			this->publish_offboard_control_mode(false, false, false, true, false);
			this->Lyapunov_based_formation_controller();
			//publish_trajectory_setpoint();
			
			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11 && vehicle_type == 2){
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(20ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr formation_error_publisher_;
	
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_position_subscriber_ ;
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscriber_ ;
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr virtual_leader_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wind_estimation_subscriber_;

	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
	
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	
	uint64_t offboard_setpoint_counter_= 0;   //!< counter for the number of setpoints sent
	
	void publish_offboard_control_mode(bool position, bool velocity, bool accerleration, bool attitude, bool bodyrate);
	void publish_trajectory_setpoint();
	
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);
	void request_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);
	
	// service states
	bool service_done_= false;
	uint8_t service_result_;

	// callback function
	void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
	void virtual_leader_callback(const std_msgs::msg::Float32MultiArray &msg);
	void vehicle_position_callback(const VehicleLocalPosition &msg);
	void vehicle_status_callback(const VehicleStatus &msg);
	void wind_estimation_callback(const std_msgs::msg::Float32MultiArray &msg);

	// follower information 
	Vector3f vehicle_position_NED = Vector3f (0.0,0.0,0.0);
	// leader information
	Vector3f leader_position_NED = Vector3f (0.0,0.0,0.0);
	float leader_course = 0.0;
	float leader_angular = 0.0;
	float leader_z_dot = 0.0;
	float leader_Vg = 0.0;
	// formation
	float gain[3] = {1.0,0.2,1.0}; // le, fe, he
	Vector3f formation_desired = Vector3f (0.0,0.0,0.0);
	Vector3f formation_error = Vector3f (0.0,0.0,0.0);
	void formation_geometry();
	void Lyapunov_based_formation_controller();
	
	// vehicle status
	unsigned int vehicle_type=1; 

	// wind information
	bool wind_est = true;
	float w_l_hat = 0.0;
	float w_f_hat = 0.0;
	float w_h_hat = 0.0;
};
// UniquePtr instead?
void FormationControl::virtual_leader_callback(const std_msgs::msg::Float32MultiArray &msg)
{	
	this->leader_position_NED << msg.data[0], msg.data[1], msg.data[2];
	this->leader_course = msg.data[3];
	this->leader_angular = msg.data[4];
	this->leader_z_dot = msg.data[5];
	this->leader_Vg = msg.data[6];
}

void FormationControl::vehicle_position_callback(const VehicleLocalPosition &msg)
{
	this->vehicle_position_NED << msg.x, msg.y, msg.z;
	// std::cout << this->vehicle_position_NED <<std::endl;
}

void FormationControl::vehicle_status_callback(const VehicleStatus &msg)
{
	this->vehicle_type = msg.vehicle_type;
}

void FormationControl::wind_estimation_callback(const std_msgs::msg::Float32MultiArray &msg)
{
	this->w_l_hat = msg.data[0];
	this->w_f_hat = msg.data[1];
	this->w_h_hat = msg.data[2];

}
void FormationControl::formation_geometry()
{	
	Matrix3f transMatrix{
		{sinf(this->leader_course), -cosf(this->leader_course), 0},
		{cosf(this->leader_course),  sinf(this->leader_course), 0},
		{0         ,  0         ,-1}
	};
	this->formation_error = transMatrix*(this->leader_position_NED - this->vehicle_position_NED) - this->formation_desired; 
	
	// send formation error
	std_msgs::msg::Float32MultiArray formationErrInfo{};
	formationErrInfo.data.push_back(this->formation_error[0]);
	formationErrInfo.data.push_back(this->formation_error[1]);
	formationErrInfo.data.push_back(this->formation_error[2]);
	this->formation_error_publisher_->publish(formationErrInfo);

}

void FormationControl::Lyapunov_based_formation_controller()
{
	float c1 = this->gain[0];
	float c2 = this->gain[1];
	float c3 = this->gain[2];
	float le = this->formation_error[0];
	float fe = this->formation_error[1];
	float he = this->formation_error[2];
	float lc = this->formation_desired[0];
	float fc = this->formation_desired[1];
	// float hc = this->formation_desired[2];
	float psi_Fc = 0.0;
	float Va_Fc = 0.0; 	
	float theta_Fc = 0.0;
	// controller
	if (this->wind_est){
		psi_Fc = atan2f(-c1*le-this->leader_angular*fc-this->w_l_hat, c2*fe-this->leader_angular*lc+this->leader_Vg+this->w_f_hat)+this->leader_course;
		Va_Fc = sqrt(pow((c1*le+this->leader_angular*fc+this->w_l_hat),2)+pow((c2*fe-this->leader_angular*lc+this->leader_Vg+this->w_f_hat),2));
		theta_Fc = asinf((c3*he-this->leader_z_dot+this->w_h_hat)/Va_Fc);
	}
	else {
		psi_Fc = atan2f(-c1*le-this->leader_angular*fc, c2*fe-this->leader_angular*lc+this->leader_Vg)+this->leader_course;
		Va_Fc = sqrt(pow((c1*le+this->leader_angular*fc),2)+pow((c2*fe-this->leader_angular*lc+this->leader_Vg),2));
		theta_Fc = asinf((c3*he-this->leader_z_dot)/Va_Fc);
	}
	
	// send control command
	VehicleAttitudeSetpoint attitude_msg{};
	tf2::Quaternion attitude_quat;
	
	attitude_quat.setRPY(0, theta_Fc, psi_Fc);
	attitude_quat.normalized();
	attitude_msg.q_d ={float(attitude_quat.w()),float(attitude_quat.x()),float(attitude_quat.y()),float(attitude_quat.z())}; // w, x, y, z
	attitude_msg.thrust_body = {NAN ,0.0, 0.0};
	attitude_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	
	this->vehicle_attitude_publisher_->publish(attitude_msg);
	this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_CHANGE_SPEED, 0, Va_Fc,-1);
	std::cout<<"Command sending..."<<std::endl;
	std::cout<<"Va_Fc : "<< Va_Fc << std::endl;
	std::cout<<"Psi_Fc : "<< psi_Fc << std::endl;
	std::cout<<"theta_Fc : "<< theta_Fc << std::endl;
	
}
/**
 * @brief Send a command to Arm the vehicle
 */
void FormationControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void FormationControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void FormationControl::publish_offboard_control_mode(bool position, bool velocity, bool accerleration, bool attitude, bool bodyrate)
{
	OffboardControlMode msg{};
	msg.position = position;
	msg.velocity = velocity;
	msg.acceleration = accerleration;
	msg.attitude = attitude;
	msg.body_rate = bodyrate;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	this->offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void FormationControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, -5.0};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void FormationControl::publish_vehicle_command(uint16_t command, float param1 , float param2, float param3, float param4, float param5 , float param6 , float param7)
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
void FormationControl::request_vehicle_command(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
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
	auto result = vehicle_command_client_->async_send_request(request, std::bind(&FormationControl::response_callback, this, std::placeholders::_1));
	RCLCPP_INFO(this->get_logger(), "Command send");
}

void FormationControl::response_callback(
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
	std::cout << "Starting formation controller node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FormationControl>());

	rclcpp::shutdown();
	return 0;
}
