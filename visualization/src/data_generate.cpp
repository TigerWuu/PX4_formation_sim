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

class DataGenerate : public rclcpp::Node
{
public:
	DataGenerate() : Node("formation_control")
	{
		// publisher
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_attitude_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		// subscriber
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		vehicle_position_subscriber_ = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&DataGenerate::vehicle_position_callback,this, _1));
		virtual_leader_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/virtual_leader_information", qos, std::bind(&DataGenerate::virtual_leader_callback, this, _1));
		wind_estimation_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/wind_estimation_information", qos, std::bind(&DataGenerate::wind_estimation_callback, this, _1));
		vehicle_status_subscriber_ = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status", qos, std::bind(&DataGenerate::vehicle_status_callback, this, _1));
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
	void virtual_leader_callback(const std_msgs::msg::Float32MultiArray &msg);
	void vehicle_position_callback(const VehicleLocalPosition &msg);
	void vehicle_attitude_callback(const VehicleAttitude &msg);
	void vehicle_airspeed_callback(const AirspeedValidated &msg);

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
void DataGenerate::virtual_leader_callback(const std_msgs::msg::Float32MultiArray &msg)
{	
	this->leader_position_NED << msg.data[0], msg.data[1], msg.data[2];
	this->leader_course = msg.data[3];
	this->leader_angular = msg.data[4];
	this->leader_z_dot = msg.data[5];
	this->leader_Vg = msg.data[6];
}

void DataGenerate::vehicle_position_callback(const VehicleLocalPosition &msg)
{
	this->vehicle_position_NED << msg.x, msg.y, msg.z;
	// std::cout << this->vehicle_position_NED <<std::endl;
}

void DataGenerate::vehicle_status_callback(const VehicleStatus &msg)
{
	this->vehicle_type = msg.vehicle_type;
}

void DataGenerate::wind_estimation_callback(const std_msgs::msg::Float32MultiArray &msg)
{
	this->w_l_hat = msg.data[0];
	this->w_f_hat = msg.data[1];
	this->w_h_hat = msg.data[2];

}

int main(int argc, char *argv[])
{
	std::cout << "Starting data generation node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DataGenerate>());

	rclcpp::shutdown();
	return 0;
}
