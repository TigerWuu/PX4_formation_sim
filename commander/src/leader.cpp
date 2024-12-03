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
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/airspeed_validated.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> //WTF?
#include <std_msgs/msg/float32_multi_array.hpp>
#include "self_msg/msg/float32_multi_array_stamped.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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

class Leader : public rclcpp::Node
{
public:
	Leader() : Node("leader")
	{	
		this->declare_parameter<int>("leader_uav_ID", 0);
		int vehicle_id = this->get_parameter("leader_uav_ID").as_int()+1;
		
		std::string uav;
		if (vehicle_id == 1){
			uav = "";
		}
		else{
	 		uav = "/px4_"+ std::to_string(this->get_parameter("leader_uav_ID").as_int());
		}

		// publisher
		leader_information_publisher_ = this->create_publisher<self_msg::msg::Float32MultiArrayStamped>("/leader_information", 10);
		// subscriber
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		vehicle_local_position_subscriber_ = this->create_subscription<VehicleLocalPosition>(uav+"/fmu/out/vehicle_local_position", qos, std::bind(&Leader::vehicle_local_position_callback,this, _1));	
		vehicle_attitude_subscriber_ = this->create_subscription<VehicleAttitude>(uav+"/fmu/out/vehicle_attitude", qos, std::bind(&Leader::vehicle_attitude_callback,this, _1));	
		vehicle_angular_velocity_subscriber_ = this->create_subscription<VehicleAngularVelocity>(uav+"/fmu/out/vehicle_angular_velocity", qos, std::bind(&Leader::vehicle_angular_velocity_callback,this, _1));	
		
		vehicle_airspeed_subscriber_ = this->create_subscription<AirspeedValidated>(uav+"/fmu/out/airspeed_validated", qos, std::bind(&Leader::vehicle_airspeed_callback,this, _1));	
		vehicle_status_subscriber_ = this->create_subscription<VehicleStatus>(uav+"/fmu/out/vehicle_status", qos, std::bind(&Leader::vehicle_status_callback, this, _1));
		
		auto timer_callback = [this]() -> void {
			// if (this->vehicle_type == 2){
				// publish leader estimation 
				this->leader_information_publish();
			//}
		};
		timer_ = this->create_wall_timer(20ms, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<self_msg::msg::Float32MultiArrayStamped>::SharedPtr leader_information_publisher_ ;
	
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_ ;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_ ;
	rclcpp::Subscription<VehicleAngularVelocity>::SharedPtr vehicle_angular_velocity_subscriber_ ;

	rclcpp::Subscription<AirspeedValidated>::SharedPtr vehicle_airspeed_subscriber_ ;
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscriber_ ; 

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	
	// callback function
	void vehicle_local_position_callback(const VehicleLocalPosition &msg);
	void vehicle_attitude_callback(const VehicleAttitude &msg);
	void vehicle_angular_velocity_callback(const VehicleAngularVelocity &msg);

	void vehicle_airspeed_callback(const AirspeedValidated &msg);
	void vehicle_status_callback(const VehicleStatus &msg);

	void leader_information_publish();
	// leader information
	float leader_x = 0.0;
	float leader_y = 0.0;
	float leader_z = 0.0;
	float leader_course = 0.0;
	float leader_angular = 0.0;
	float leader_z_dot = 0.0;
	float leader_Vg = 0.0;
	// vehicle information
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	float Va = 0.0;

	// vehicle status, 1 for multirotor, 2 for fixed-wing
	unsigned int vehicle_type=1; 
};
// UniquePtr instead?
void Leader::leader_information_publish()
{
	self_msg::msg::Float32MultiArrayStamped leaderInfo;
	
	leaderInfo.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	leaderInfo.array.data.push_back(this->leader_x);
	leaderInfo.array.data.push_back(this->leader_y);
	leaderInfo.array.data.push_back(this->leader_z);
	leaderInfo.array.data.push_back(this->leader_course);
	leaderInfo.array.data.push_back(this->leader_angular);
	leaderInfo.array.data.push_back(this->leader_z_dot);
	leaderInfo.array.data.push_back(this->leader_Vg);
	this->leader_information_publisher_->publish(leaderInfo);
}

void Leader::vehicle_local_position_callback(const VehicleLocalPosition &msg)
{	
	this->leader_x = msg.x;
	this->leader_y = msg.y;
	this->leader_z = msg.z;
	this->leader_z_dot = msg.z_deriv;
	this->leader_Vg = sqrt(pow(msg.vx,2)+pow(msg.vy,2)+pow(msg.vz,2));
	this->leader_course = atan2f(msg.vy, msg.vx);
}

void Leader::vehicle_attitude_callback(const VehicleAttitude &msg)
{
	tf2::Quaternion current_att_quat(msg.q[1], msg.q[2], msg.q[3], msg.q[0]); // xyzw
	tf2::Matrix3x3 m(current_att_quat);
	m.getRPY(this->roll,this->pitch, this->yaw);
}

void Leader::vehicle_angular_velocity_callback(const VehicleAngularVelocity &msg)
{
	this->leader_angular = msg.xyz[2];	
}
void Leader::vehicle_airspeed_callback(const AirspeedValidated &msg)
{
	this->Va = msg.true_airspeed_m_s;
}
void Leader::vehicle_status_callback(const VehicleStatus &msg)
{
	vehicle_type = msg.vehicle_type;
}

int main(int argc, char *argv[])
{
	std::cout << "Starting leader node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Leader>());

	rclcpp::shutdown();
	return 0;
}
