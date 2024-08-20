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

class VirtualLeader : public rclcpp::Node
{
public:
	VirtualLeader() : Node("virtual_leader")
	{
		// publisher
		virtual_leader_information_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("virtual_leader_information", 10);
		
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
		
		this->t0 = this->get_clock()->now().seconds(); // s 
		this->trajectory = 'C';
		this->timer_ = this->create_wall_timer(100ms, std::bind(&VirtualLeader::virtual_leader_information, this));
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr virtual_leader_information_publisher_;
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	char trajectory; //Circular path 
	double t0; // s 
	// leader information
	void virtual_leader_information();
	
	float x_L = 0.0;
	float y_L = 0.0;
	float z_L = 0.0;
	float course_L = 0.0;
	float course_L_dot = 0.0;
	float z_L_dot = 0.0;
	float Vg_L = 18.0;
};
// UniquePtr instead?
void VirtualLeader::virtual_leader_information()
{	
	std::cout.precision(10);
	double t_now = this->get_clock()->now().seconds();
	double t = t_now - this->t0;
	std::cout << "Time 0: " << this->t0 << std::endl;
	std::cout << "Time now: " << t_now << std::endl;
	std::cout << "Time : " << t << std::endl;
	std_msgs::msg::Float32MultiArray leaderInfo;
	float radii = 200.0;
	float omega = 0.0;
	float x_L_dot = 0.0;
	float y_L_dot = 0.0;
	float x_L_dotdot = 0.0;
	float y_L_dotdot = 0.0;

	switch (this->trajectory)
	{
		case 'C':
			omega = this->Vg_L/radii;
			this->x_L = radii*sin(omega*t);
			this->y_L = radii*cos(omega*t);
			this->z_L = -90.0;
			x_L_dot = radii*omega*cos(omega*t);
			y_L_dot = -radii*omega*sin(omega*t);
			this->z_L_dot = 0.0;
			x_L_dotdot = -radii*pow(omega,2)*sin(omega*t);
			y_L_dotdot = -radii*pow(omega,2)*cos(omega*t);
			this->course_L = atan2f(y_L_dot, x_L_dot);
			this->course_L_dot = (y_L_dotdot*x_L_dot-x_L_dotdot*y_L_dot)/(pow(x_L_dot,2)+pow(y_L_dot,2));
			break;
		case 'L':
			std::cout<< "Straight line path !!!"<<std::endl;
			break;
		default:
			std::cout<< "No trajectory !!!"<<std::endl;
			break;
	}
	leaderInfo.data.push_back(this->x_L);
	leaderInfo.data.push_back(this->y_L);
	leaderInfo.data.push_back(this->z_L);
	leaderInfo.data.push_back(this->course_L);
	leaderInfo.data.push_back(this->course_L_dot);
	leaderInfo.data.push_back(this->z_L_dot);
	leaderInfo.data.push_back(this->Vg_L);
	this->virtual_leader_information_publisher_->publish(leaderInfo);
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
