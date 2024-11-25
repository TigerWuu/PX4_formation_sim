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

class WindObserver : public rclcpp::Node
{
public:
	WindObserver() : Node("wind_observer")
	{	
		this->declare_parameter<int>("uav_ID", 0);
		int vehicle_id = this->get_parameter("uav_ID").as_int()+1;
		
		this->declare_parameter<double>("L", 1.0);
		this->L = this->get_parameter("L").as_double();
		this->declare_parameter<double>("L2", 1.0);
		this->L2 = this->get_parameter("L2").as_double();
		
		std::string uav;
		if (vehicle_id == 1){
			uav = "";
		}
		else{
	 		uav = "/px4_"+ std::to_string(this->get_parameter("uav_ID").as_int());
		}

		// publisher
		wind_estimation_publisher_ = this->create_publisher<self_msg::msg::Float32MultiArrayStamped>(uav+"/wind_estimation_information", 10);
		// subscriber
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		virtual_leader_subscriber_ = this->create_subscription<self_msg::msg::Float32MultiArrayStamped>("/virtual_leader_information", qos, std::bind(&WindObserver::virtual_leader_callback, this, _1));
		vehicle_attitude_subscriber_ = this->create_subscription<VehicleAttitude>(uav+"/fmu/out/vehicle_attitude", qos, std::bind(&WindObserver::vehicle_attitude_callback,this, _1));	
		vehicle_airspeed_subscriber_ = this->create_subscription<AirspeedValidated>(uav+"/fmu/out/airspeed_validated", qos, std::bind(&WindObserver::vehicle_airspeed_callback,this, _1));	
		vehicle_status_subscriber_ = this->create_subscription<VehicleStatus>(uav+"/fmu/out/vehicle_status", qos, std::bind(&WindObserver::vehicle_status_callback, this, _1));
		wind_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/data/wind_true", qos, std::bind(&WindObserver::wind_callback, this, _1));
		formation_subscriber_ = this->create_subscription<self_msg::msg::Float32MultiArrayStamped>(uav+"/data/formation_error", qos, std::bind(&WindObserver::formation_info_callback, this, _1));
		
		auto timer_callback = [this]() -> void {
			// if (this->vehicle_type == 1){
				float time_interval = 0.02;
				float sigma_l = this->le_hat - this->le; 
				float sigma_f = this->fe_hat - this->fe; 
				float sigma_h = this->he_hat - this->he; 
				
				float sigma_al = this->w_l2_hat - this->w_l_hat; 
				float sigma_af = this->w_f2_hat - this->w_f_hat; 
				float sigma_ah = this->w_h2_hat - this->w_h_hat; 

				// wind estimation
				this->wind_estimation(sigma_l, sigma_f, sigma_h, time_interval);	
				// wind acceleration estimation
				this->wind_accel_estimation(sigma_al, sigma_af, sigma_ah, time_interval);
				// position estimation
				this->geometry_error_estimation(sigma_l, sigma_f, sigma_h, time_interval);	
				// wind 2 estimation
				this->wind_2_estimation(sigma_al, sigma_af, sigma_ah, time_interval);
				// publish wind estimation (formation frame)
				this->wind_information_publish();
			// }
		};
		timer_ = this->create_wall_timer(20ms, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<self_msg::msg::Float32MultiArrayStamped>::SharedPtr wind_estimation_publisher_ ;
	
	rclcpp::Subscription<self_msg::msg::Float32MultiArrayStamped>::SharedPtr virtual_leader_subscriber_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_position_subscriber_ ;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_ ;
	rclcpp::Subscription<AirspeedValidated>::SharedPtr vehicle_airspeed_subscriber_ ;
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscriber_ ; 
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wind_subscriber_;
	rclcpp::Subscription<self_msg::msg::Float32MultiArrayStamped>::SharedPtr formation_subscriber_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	
	void wind_information_publish();
	void coordinate_transformation(); // NED frame to formation frame 
	// callback function
	void virtual_leader_callback(const self_msg::msg::Float32MultiArrayStamped &msg);
	void vehicle_attitude_callback(const VehicleAttitude &msg);
	void vehicle_airspeed_callback(const AirspeedValidated &msg);
	void vehicle_status_callback(const VehicleStatus &msg);
	void wind_callback(const std_msgs::msg::Float32MultiArray &msg);
	void formation_info_callback(const self_msg::msg::Float32MultiArrayStamped &msg);

	// observer function
	static int sgn(float sigma);
	void wind_accel_estimation(float sigma_al, float sigma_af, float sigma_ah, float time_int);
	void wind_2_estimation(float sigma_al, float sigma_af, float sigma_ah, float time_int);
	void wind_estimation(float sigma_l, float sigma_f, float sigma_h, float time_int);
	void geometry_error_estimation(float sigma_l, float sigma_f, float sigma_h, float time_int);
	// leader information
	Vector3f leader_position_NED = Vector3f (0.0,0.0,0.0);
	float leader_course = 0.0;
	float leader_angular = 0.0;
	float leader_z_dot = 0.0;
	float leader_Vg = 0.0;
	// vehicle information
	float le = 0.0;
	float fe = 0.0;
	float he = 0.0;
	
	float lc = 0.0;
	float fc = 0.0;
	float hc = 0.0;

	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	float Va = 0.0;
	// observer parameters
	float L = 10.0;
	float L2 = 1.0;
	// observer variables
	float le_hat = 0.0; 
	float fe_hat = 0.0; 
	float he_hat = 0.0; 
	
	float w_l_hat = 0.0;
	float w_f_hat = 0.0;
	float w_h_hat = 0.0;
	// acceleration observer variables
	float w_l2_hat = 0.0;
	float w_f2_hat = 0.0;
	float w_h2_hat = 0.0;

	float wa_l_hat = 0.0;
	float wa_f_hat = 0.0;
	float wa_h_hat = 0.0;

	// true wind information	
	float w_n = 0.0;
	float w_e = 0.0;
	float w_d = 0.0;
	
	float w_l = 0.0;
	float w_f = 0.0;
	float w_h = 0.0;

	// vehicle status
	unsigned int vehicle_type=1; 
};
// UniquePtr instead?
void WindObserver::wind_information_publish()
{
	self_msg::msg::Float32MultiArrayStamped windEstInfo;
	
	windEstInfo.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	// second observer
	windEstInfo.array.data.push_back(this->w_l2_hat);
	windEstInfo.array.data.push_back(this->w_f2_hat);
	windEstInfo.array.data.push_back(this->w_h2_hat);
	windEstInfo.array.data.push_back(this->wa_l_hat);
	windEstInfo.array.data.push_back(this->wa_f_hat);
	windEstInfo.array.data.push_back(this->wa_h_hat);
	// first observer
	windEstInfo.array.data.push_back(this->w_l_hat);
	windEstInfo.array.data.push_back(this->w_f_hat);
	windEstInfo.array.data.push_back(this->w_h_hat);
	windEstInfo.array.data.push_back(this->le_hat);
	windEstInfo.array.data.push_back(this->fe_hat);
	windEstInfo.array.data.push_back(this->he_hat);
	windEstInfo.array.data.push_back(this->w_l);
	windEstInfo.array.data.push_back(this->w_f);
	windEstInfo.array.data.push_back(this->w_h);
	// windEstInfo.array.data.push_back(this->w_n);
	// windEstInfo.array.data.push_back(this->w_e);
	// windEstInfo.array.data.push_back(this->w_d);
	this->wind_estimation_publisher_->publish(windEstInfo);
}
void WindObserver::wind_callback(const std_msgs::msg::Float32MultiArray &msg)
{
	this->w_n = msg.data[1];
	this->w_e = msg.data[0];
	this->w_d = -msg.data[2];
	
	// ned to frd
	Matrix3f trans_ned_frd{
		{cosf(this->leader_course), -sinf(this->leader_course), 0},
		{sinf(this->leader_course),  cosf(this->leader_course), 0},
		{0, 0, 1}
	};
	
	Vector3f wind_ned = Vector3f (this->w_n,this->w_e,this->w_d);
	Vector3f wind_frd = trans_ned_frd.transpose()*wind_ned;
	this->w_l = wind_frd[1];
	this->w_f = -wind_frd[0];
	this->w_h = wind_frd[2];


}
void WindObserver::virtual_leader_callback(const self_msg::msg::Float32MultiArrayStamped &msg)
{	
	this->leader_position_NED << msg.array.data[0], msg.array.data[1], msg.array.data[2];
	this->leader_course = msg.array.data[3];
	this->leader_angular = msg.array.data[4];
	this->leader_z_dot = msg.array.data[5];
	this->leader_Vg = msg.array.data[6];
}

void WindObserver::formation_info_callback(const self_msg::msg::Float32MultiArrayStamped &msg)
{
	this->le = msg.array.data[0];
	this->fe = msg.array.data[1];
	this->he = msg.array.data[2];
	// desired formation
	this->lc = msg.array.data[3];
	this->fc = msg.array.data[4];
	this->hc = msg.array.data[5];
}

void WindObserver::vehicle_attitude_callback(const VehicleAttitude &msg)
{
	tf2::Quaternion current_att_quat(msg.q[1], msg.q[2], msg.q[3], msg.q[0]); // xyzw
	tf2::Matrix3x3 m(current_att_quat);
	m.getRPY(this->roll,this->pitch, this->yaw);
}

void WindObserver::vehicle_airspeed_callback(const AirspeedValidated &msg)
{
	this->Va = msg.true_airspeed_m_s;
	// float Va_ias = msg.indicated_airspeed_m_s;
	// float Va_cas = msg.calibrated_airspeed_m_s;
}
void WindObserver::vehicle_status_callback(const VehicleStatus &msg)
{
	vehicle_type = msg.vehicle_type;
}
void WindObserver::wind_accel_estimation(float sigma_al, float sigma_af, float sigma_ah, float time_int)
{
	float lambda0 = 1.1*this->L2;
	float wj_l_hat = -lambda0*sgn(sigma_al);
	float wj_f_hat = -lambda0*sgn(sigma_af);
	float wj_h_hat = -lambda0*sgn(sigma_ah);

	this->wa_l_hat += wj_l_hat*time_int;
	this->wa_f_hat += wj_f_hat*time_int;
	this->wa_h_hat += wj_h_hat*time_int;
}

void WindObserver::wind_2_estimation(float sigma_al, float sigma_af, float sigma_ah, float time_int)
{
	float lambda1 = 1.5*sqrt(this->L2);
	float wa2_l_hat = -lambda1*sqrt(abs(sigma_al))*sgn(sigma_al)+this->wa_l_hat;
	float wa2_f_hat = -lambda1*sqrt(abs(sigma_af))*sgn(sigma_af)+this->wa_f_hat;
	float wa2_h_hat = -lambda1*sqrt(abs(sigma_ah))*sgn(sigma_ah)+this->wa_h_hat;

	this->w_l2_hat += wa2_l_hat*time_int;
	this->w_f2_hat += wa2_f_hat*time_int;
	this->w_h2_hat += wa2_h_hat*time_int;
}
void WindObserver::wind_estimation(float sigma_l, float sigma_f, float sigma_h, float time_int)
{
	float lambda0 = 1.1*this->L;
	float wa1_l_hat = -lambda0*sgn(sigma_l);
	float wa1_f_hat = -lambda0*sgn(sigma_f);
	float wa1_h_hat = -lambda0*sgn(sigma_h);

	this->w_l_hat += wa1_l_hat*time_int;
	this->w_f_hat += wa1_f_hat*time_int;
	this->w_h_hat += wa1_h_hat*time_int;
}

void WindObserver::geometry_error_estimation(float sigma_l, float sigma_f, float sigma_h, float time_int)
{
	float lambda1 = 1.5*sqrt(this->L);
	float le_dot_hat = this->Va*sinf(this->yaw-this->leader_course) + this->leader_angular*(this->fe+this->fc)-lambda1*sqrt(abs(sigma_l))*sgn(sigma_l)+this->w_l_hat;
	float fe_dot_hat = this->leader_Vg-this->Va*cosf(this->yaw-this->leader_course) - this->leader_angular*(this->le+this->lc)-lambda1*sqrt(abs(sigma_f))*sgn(sigma_f)+this->w_f_hat;
	float he_dot_hat = -this->leader_z_dot-this->Va*sinf(this->pitch)-lambda1*sqrt(abs(sigma_h))*sgn(sigma_h)+this->w_h_hat;
	
	this->le_hat += le_dot_hat*time_int; 
	this->fe_hat += fe_dot_hat*time_int; 
	this->he_hat += he_dot_hat*time_int; 
	// std::cout << "sigma_l: " << sigma_l << std::endl;
	// std::cout << "sigma_f: " << sigma_f << std::endl;
	// std::cout << "sigma_h: " << sigma_h << std::endl;
}


int WindObserver::sgn(float sigma)
{
	if (sigma > 0){
		return 1;
	}
	else if (sigma < 0){
		return -1;
	}
	else{
		return 0;
	}
}

int main(int argc, char *argv[])
{
	std::cout << "Starting wind observer node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WindObserver>());

	rclcpp::shutdown();
	return 0;
}
