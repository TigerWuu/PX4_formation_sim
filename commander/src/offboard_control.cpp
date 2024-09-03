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
#define _USE_MATH_DEFINES
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/airspeed_validated.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <stdint.h>

#include <std_msgs/msg/float32_multi_array.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

using std::placeholders::_1;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_attitude_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		
		cmd_info_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("command_information", 10);
		// subscriber
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		vehicle_attitude_subscriber_ = this->create_subscription<VehicleAttitude>("/fmu/out/vehicle_attitude", qos, std::bind(&OffboardControl::vehicle_attitude_callback,this, _1));	
		vehicle_airspeed_subscriber_ = this->create_subscription<AirspeedValidated>("/fmu/out/airspeed_validated", qos, std::bind(&OffboardControl::vehicle_airspeed_callback,this, _1));	
		offboard_setpoint_counter_ = 0;

		this->Va_set = 22.0;
		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				// this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			this->publish_offboard_control_mode();
			// this->publish_trajectory_setpoint();
			this->publish_attitude_setpoint();
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_CHANGE_SPEED, 0, this->Va_set ,-1);
			// std::cout <<"changing speed : "<< this->Va << std::endl;

			std::cout<<"damn!!" << std::endl;
			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(20ms, timer_callback); // 50 Hz
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_info_publisher_;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_ ;
	rclcpp::Subscription<AirspeedValidated>::SharedPtr vehicle_airspeed_subscriber_ ;
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_attitude_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3= 0.0, float param4= 0.0, float param5= 0.0, float param6= 0.0, float param7= 0.0);
	void vehicle_attitude_callback(const VehicleAttitude &msg);
	void vehicle_airspeed_callback(const AirspeedValidated &msg);
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	float Va = 0.0;
	float Va_set = 0.0;
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void OffboardControl::vehicle_attitude_callback(const VehicleAttitude &msg)
{
	tf2::Quaternion current_att_quat(msg.q[1], msg.q[2], msg.q[3], msg.q[0]); // xyzw
	tf2::Matrix3x3 m(current_att_quat);
	m.getRPY(roll,pitch, yaw);
}

void OffboardControl::vehicle_airspeed_callback(const AirspeedValidated &msg)
{
	Va = msg.true_airspeed_m_s;
	// float Va_ias = msg.indicated_airspeed_m_s;
	// float Va_cas = msg.calibrated_airspeed_m_s;
	// std::cout << "ias : "<<Va_ias << std::endl;
	// std::cout << "cas : "<<Va_cas << std::endl;

}
/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = true;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	// msg.position = {2000.0, 2000.0, -100.0};
	msg.position = {NAN, NAN, NAN};
	// msg.velocity = {0.0, 0.0, -0.5};
	msg.yaw = M_PI/3; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_attitude_setpoint()
{
	VehicleAttitudeSetpoint attitude_msg;
	std_msgs::msg::Float32MultiArray cmdInfo;
	tf2::Quaternion attitude_quat;
	float roll_set = M_PI/36;
	float pitch_set = 0; // 5 deg
	float yaw_set = M_PI/3; // 60 deg
	// float k_y = 0.2;
	// float yaw_err = yaw_set-yaw;
	// while (yaw_err < -M_PI){
	// 	yaw_err += 2*M_PI;
	// }
	// while (yaw_err > M_PI){
	// 	yaw_err -= 2*M_PI;
	// }
	// roll_set = (yaw_err)*k_y;
	attitude_quat.setRPY(roll_set, pitch_set, yaw_set);
	attitude_quat.normalized();
	// attitude_quat.setRPY(0.0, M_PI/18, 0.0);
	attitude_msg.q_d ={float(attitude_quat.w()),float(attitude_quat.x()),float(attitude_quat.y()),float(attitude_quat.z())}; // w, x, y, z
	attitude_msg.thrust_body = {NAN ,0.0, 0.0};
	std::cout << "pitch_err : " << pitch_set - pitch << std::endl;
	std::cout << "yaw_err : " << yaw_set - yaw << std::endl;
	std::cout << "Va_err : " << this->Va_set - this->Va << std::endl;
	attitude_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	this->vehicle_attitude_publisher_->publish(attitude_msg);
	cmdInfo.data.push_back(pitch_set);
	cmdInfo.data.push_back(this->pitch);
	cmdInfo.data.push_back(pitch_set-this->pitch);
	cmdInfo.data.push_back(yaw_set);
	cmdInfo.data.push_back(this->yaw);
	cmdInfo.data.push_back(yaw_set-this->yaw);
	cmdInfo.data.push_back(this->Va_set);
	cmdInfo.data.push_back(this->Va);
	cmdInfo.data.push_back(this->Va_set-this->Va);

	this->cmd_info_publisher_->publish(cmdInfo);
}
/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1 , float param2, float param3, float param4, float param5 , float param6 , float param7)
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

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
