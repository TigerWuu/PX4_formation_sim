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
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp> //WTF?
#include <std_msgs/msg/float32_multi_array.hpp>
#include "self_msg/msg/float32_multi_array_stamped.hpp"

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

class VirtualLeader : public rclcpp::Node
{
public:
	VirtualLeader() : Node("virtual_leader")
	{			
		// subscriber
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
		vehicle_status_subscriber_ = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status", qos, std::bind(&VirtualLeader::vehicle_status_callback, this, _1));
		// define trajectory
		this->declare_parameter<std::string>("trajectory", "L");
		this->declare_parameter<double>("direction", 0.0);
		this->declare_parameter<double>("radii", 200.0);
		this->trajectory = this->get_parameter("trajectory").as_string()[0];
		this->dir = this->get_parameter("direction").as_double();
		this->radii = this->get_parameter("radii").as_double();

		// publisher
		virtual_leader_information_publisher_ = this->create_publisher<self_msg::msg::Float32MultiArrayStamped>("/virtual_leader_information", 10);
		
		// while (this->vehicle_type == 1){
		// 	std::cout << "Waiting for FW ..." <<std::endl;
		//}
		this->t0 = this->get_clock()->now().seconds(); // s 
		this->timer_ = this->create_wall_timer(20ms, std::bind(&VirtualLeader::virtual_leader_information, this));
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<self_msg::msg::Float32MultiArrayStamped>::SharedPtr virtual_leader_information_publisher_;
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscriber_ ;
	
	char trajectory; //Circular path 
	double t0; // s 
	// leader information
	void virtual_leader_information();
	void vehicle_status_callback(const VehicleStatus &msg);
	
	float x_L = 0.0;
	float y_L = 0.0;
	float z_L = 0.0;
	float course_L = 0.0;
	float course_L_dot = 0.0;
	float z_L_dot = 0.0;
	float Vg_L = 18.0;
	float dir = 0.0;
	float radii = 200.0;
	// vehicle status
	unsigned int vehicle_type = 1; 
	unsigned int in_transition_fw = 0; 
};
// UniquePtr instead?
void VirtualLeader::virtual_leader_information()
{	
	std::cout.precision(15);
	double t_now = this->get_clock()->now().seconds();
	double t = t_now - this->t0;
	std::cout << "Time 0: " << this->t0 << std::endl;
	std::cout << "Time now: " << t_now << std::endl;
	std::cout << "Time : " << t << std::endl;
	self_msg::msg::Float32MultiArrayStamped leaderInfo;
	float omega = 0.0;
	float x_L_dot = 0.0;
	float y_L_dot = 0.0;
	float x_L_dotdot = 0.0;
	float y_L_dotdot = 0.0;

	switch (this->trajectory)
	{
		case 'C':
			omega = this->Vg_L/this->radii;
			this->x_L = this->radii*cosf(omega*t) - this->radii;
			this->y_L = this->radii*sinf(omega*t);
			this->z_L = -90.0;
			x_L_dot = -this->radii*omega*sinf(omega*t);
			y_L_dot = this->radii*omega*cosf(omega*t);
			this->z_L_dot = 0.0;
			x_L_dotdot = -this->radii*pow(omega,2)*cosf(omega*t);
			y_L_dotdot = -this->radii*pow(omega,2)*sinf(omega*t);
			this->course_L = atan2f(y_L_dot, x_L_dot);
			this->course_L_dot = (y_L_dotdot*x_L_dot-x_L_dotdot*y_L_dot)/(pow(x_L_dot,2)+pow(y_L_dot,2));
			break;
		case 'L':
			this->x_L = this->Vg_L*t*cosf(this->dir);
			this->y_L = this->Vg_L*t*sinf(this->dir);
			this->z_L = -90.0;
			x_L_dot = this->Vg_L*cosf(this->dir);
			y_L_dot = this->Vg_L*sinf(this->dir);
			this->z_L_dot = 0.0;
			this->course_L = atan2f(y_L_dot, x_L_dot);
			this->course_L_dot = 0;
		
			// std::cout<< "Straight line path !!!"<<std::endl;
			break;
		default:
			std::cout<< "No trajectory !!!"<<std::endl;
			break;
	}
	leaderInfo.timestamp = t_now*1e6;
	leaderInfo.array.data.push_back(this->x_L);
	leaderInfo.array.data.push_back(this->y_L);
	leaderInfo.array.data.push_back(this->z_L);
	leaderInfo.array.data.push_back(this->course_L);
	leaderInfo.array.data.push_back(this->course_L_dot);
	leaderInfo.array.data.push_back(this->z_L_dot);
	leaderInfo.array.data.push_back(this->Vg_L);
	this->virtual_leader_information_publisher_->publish(leaderInfo);
}
void VirtualLeader::vehicle_status_callback(const VehicleStatus &msg)
{
	this->vehicle_type = msg.vehicle_type;
	this->in_transition_fw = msg.in_transition_to_fw;
}

int main(int argc, char *argv[])
{
	std::cout << "Virtual leader working..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VirtualLeader>());

	rclcpp::shutdown();
	return 0;
}
