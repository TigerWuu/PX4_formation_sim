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

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/airspeed_validated.hpp>

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

class FormationControl : public rclcpp::Node
{
public:
	FormationControl() : Node("formation_control")
	{	
		// define UAV number
		this->declare_parameter<int>("uav_ID", 0);
		this->declare_parameter<std::string>("wind_com", "com_off");
		this->vehicle_id = this->get_parameter("uav_ID").as_int()+1;
		this->wind_com = this->get_parameter("wind_com").as_string();
		std::string uav;
			
		if (this->vehicle_id == 1){
			uav = "";
		}
		else{
	 		uav = "/px4_"+ std::to_string(this->get_parameter("uav_ID").as_int());
		}
		
		// publisher
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(uav+"/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(uav+"/fmu/in/trajectory_setpoint", 10);
		vehicle_attitude_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>(uav+"/fmu/in/vehicle_attitude_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(uav+"/fmu/in/vehicle_command", 10);
		// publisher for plot
		formation_error_publisher_ = this->create_publisher<self_msg::msg::Float32MultiArrayStamped>(uav+"/data/formation_error", 10);
		attitude_publisher_ = this->create_publisher<self_msg::msg::Float32MultiArrayStamped>(uav+"/data/attitude", 10);
		control_inputs_publisher_ = this->create_publisher<self_msg::msg::Float32MultiArrayStamped>(uav+"/data/control_inputs", 10);
		// subscriber
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		vehicle_position_subscriber_ = this->create_subscription<VehicleLocalPosition>(uav+"/fmu/out/vehicle_local_position", qos, std::bind(&FormationControl::vehicle_position_callback,this, _1));
		virtual_leader_subscriber_ = this->create_subscription<self_msg::msg::Float32MultiArrayStamped>("/virtual_leader_information", qos, std::bind(&FormationControl::virtual_leader_callback, this, _1));
		wind_estimation_subscriber_ = this->create_subscription<self_msg::msg::Float32MultiArrayStamped>(uav+"/wind_estimation_information", qos, std::bind(&FormationControl::wind_estimation_callback, this, _1));
		vehicle_status_subscriber_ = this->create_subscription<VehicleStatus>(uav+"/fmu/out/vehicle_status", qos, std::bind(&FormationControl::vehicle_status_callback, this, _1));
		// subsriber for plot
		vehicle_attitude_subscriber_ = this->create_subscription<VehicleAttitude>(uav+"/fmu/out/vehicle_attitude", qos, std::bind(&FormationControl::vehicle_attitude_callback,this, _1));
		vehicle_airspeed_subscriber_ = this->create_subscription<AirspeedValidated>(uav+"/fmu/out/airspeed_validated", qos, std::bind(&FormationControl::vehicle_airspeed_callback,this, _1));	

		// client
		vehicle_command_client_ = this->create_client<px4_msgs::srv::VehicleCommand>(uav+"/fmu/vehicle_command");
		// transition form MR to FW
		while (!vehicle_command_client_->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
		}
	
		auto timer_callback = [this]() -> void {
			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				this->arm();
			}
			if (this->vehicle_type == 1 && !this->in_transition_fw && std::fabs(this->z0-this->vehicle_position_NED[2]) > 0.5){
				// takeoff to desired altitude
				this->publish_offboard_control_mode(true, false, false, false, false);
				this->publish_trajectory_setpoint();
			}
			else{
				
				if (this->vehicle_type == 1){
					this->request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, 4, 1);
				}
				else{
					// calculate the formation geometry
					this->formation_geometry();
					// offboard_control_mode needs to be paired with trajectory_setpoint
					this->publish_offboard_control_mode(false, false, false, true, false);
					if (this->vehicle_type == 2){
						this->Lyapunov_based_formation_controller();
					}
				}
			}

			
			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11){
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(20ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;
	// pub
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	// pub for plot
	rclcpp::Publisher<self_msg::msg::Float32MultiArrayStamped>::SharedPtr formation_error_publisher_;
	rclcpp::Publisher<self_msg::msg::Float32MultiArrayStamped>::SharedPtr attitude_publisher_;
	rclcpp::Publisher<self_msg::msg::Float32MultiArrayStamped>::SharedPtr control_inputs_publisher_;
	// sub
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_position_subscriber_ ;
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscriber_ ;
	rclcpp::Subscription<self_msg::msg::Float32MultiArrayStamped>::SharedPtr virtual_leader_subscriber_;
	rclcpp::Subscription<self_msg::msg::Float32MultiArrayStamped>::SharedPtr wind_estimation_subscriber_;
	// sub for plot
	rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_ ;
	rclcpp::Subscription<AirspeedValidated>::SharedPtr vehicle_airspeed_subscriber_ ;

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
	void virtual_leader_callback(const self_msg::msg::Float32MultiArrayStamped &msg);
	void vehicle_position_callback(const VehicleLocalPosition &msg);
	void vehicle_status_callback(const VehicleStatus &msg);
	void wind_estimation_callback(const self_msg::msg::Float32MultiArrayStamped &msg);

	// follower information 
	Vector3f vehicle_position_NED = Vector3f (0.0,0.0,0.0);
	// leader information
	Vector3f leader_position_NED = Vector3f (0.0,0.0,0.0);
	float leader_course = 0.0;
	float leader_angular = 0.0;
	float leader_z_dot = 0.0;
	float leader_Vg = 0.0;
	// formation
	float gain[6] = {0.1, 0.1, 1.0, 1.0, 0.5, 3.0}; // le, fe, he, Va_e, psi_e, theta_e
	Vector3f formation_desired = Vector3f (2.5,14.0,0.0);
	// Vector3f formation_desired = Vector3f (0.0,0.0,0.0);
	Vector3f formation_error = Vector3f (0.0,0.0,0.0);
	void formation_geometry();
	void Lyapunov_based_formation_controller();
	
	// vehicle status
	unsigned int vehicle_type = 1; 
	unsigned int in_transition_fw = 0; 
 	int vehicle_id = 1;
	float z0 = -20;
	// wind information
	std::string wind_com = "";
	float w_l_hat = 0.0;
	float w_f_hat = 0.0;
	float w_h_hat = 0.0;
	float wa_l_hat = 0.0;
	float wa_f_hat = 0.0;
	float wa_h_hat = 0.0;
	float w_l2_hat = 0.0;
	float w_f2_hat = 0.0;
	float w_h2_hat = 0.0;
	float w_l = 0.0;
	float w_f = 0.0;
	float w_h = 0.0;

	
	// plot
	void vehicle_attitude_callback(const VehicleAttitude &msg);
	void vehicle_airspeed_callback(const AirspeedValidated &msg);

	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;

	float Va = 0.0;

};
// UniquePtr instead?
// for plot
void FormationControl::vehicle_attitude_callback(const VehicleAttitude &msg)
{
	tf2::Quaternion current_att_quat(msg.q[1], msg.q[2], msg.q[3], msg.q[0]); // xyzw
	tf2::Matrix3x3 m(current_att_quat);
	m.getRPY(this->roll, this->pitch, this->yaw);
	
	// self_msg::msg::Float32MultiArrayStamped attitudeInfo{};
	// attitudeInfo.timestamp = this->get_clock()->now().nanoseconds()/1000;
	// attitudeInfo.array.data.push_back(this->roll);
	// attitudeInfo.array.data.push_back(this->pitch);
	// attitudeInfo.array.data.push_back(this->yaw);
	// this->attitude_publisher_->publish(attitudeInfo);

}

void FormationControl::vehicle_airspeed_callback(const AirspeedValidated &msg)
{
	Va = msg.true_airspeed_m_s;
	// float Va_ias = msg.indicated_airspeed_m_s;
	// float Va_cas = msg.calibrated_airspeed_m_s;

}

void FormationControl::virtual_leader_callback(const self_msg::msg::Float32MultiArrayStamped &msg)
{	
	this->leader_position_NED << msg.array.data[0], msg.array.data[1], msg.array.data[2];
	this->leader_course = msg.array.data[3];
	this->leader_angular = msg.array.data[4];
	this->leader_z_dot = msg.array.data[5];
	this->leader_Vg = msg.array.data[6];
}

void FormationControl::vehicle_position_callback(const VehicleLocalPosition &msg)
{
	this->vehicle_position_NED << msg.x, msg.y, msg.z;
	// std::cout << this->vehicle_position_NED <<std::endl;
}

void FormationControl::vehicle_status_callback(const VehicleStatus &msg)
{
	this->vehicle_type = msg.vehicle_type;
	this->in_transition_fw = msg.in_transition_to_fw;
}

void FormationControl::wind_estimation_callback(const self_msg::msg::Float32MultiArrayStamped &msg)
{
	// wind_observer_i
	this->w_l_hat = msg.array.data[0];
	this->w_f_hat = msg.array.data[1];
	this->w_h_hat = msg.array.data[2];
	
	this->w_l = msg.array.data[6];
	this->w_f = msg.array.data[7];
	this->w_h = msg.array.data[8];
	
	// wind_observer_g
	this->w_l2_hat = msg.array.data[0];
	this->w_f2_hat = msg.array.data[1];
	this->w_h2_hat = msg.array.data[2];
	this->wa_l_hat = msg.array.data[3];
	this->wa_f_hat = msg.array.data[4];
	this->wa_h_hat = msg.array.data[5];
	this->w_l_hat = msg.array.data[6];
	this->w_f_hat = msg.array.data[7];
	this->w_h_hat = msg.array.data[8];

	this->w_l = msg.array.data[12];
	this->w_f = msg.array.data[13];
	this->w_h = msg.array.data[14];

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
	self_msg::msg::Float32MultiArrayStamped formationErrInfo{};
	formationErrInfo.timestamp = this->get_clock()->now().nanoseconds()/1000;
	formationErrInfo.array.data.push_back(this->formation_error[0]);
	formationErrInfo.array.data.push_back(this->formation_error[1]);
	formationErrInfo.array.data.push_back(this->formation_error[2]);
	formationErrInfo.array.data.push_back(this->formation_desired[0]);
	formationErrInfo.array.data.push_back(this->formation_desired[1]);
	formationErrInfo.array.data.push_back(this->formation_desired[2]);
	this->formation_error_publisher_->publish(formationErrInfo);

}

void FormationControl::Lyapunov_based_formation_controller()
{
	float c1 = this->gain[0];
	float c2 = this->gain[1];
	float c3 = this->gain[2];
	float c4 = this->gain[3];
	float c5 = this->gain[4];
	float c6 = this->gain[5];

	float le = this->formation_error[0];
	float fe = this->formation_error[1];
	float he = this->formation_error[2];
	float lc = this->formation_desired[0];
	float fc = this->formation_desired[1];
	// float hc = this->formation_desired[2];
	// desired states
	float psi_Fd = 0.0;
	float Va_Fd = 0.0; 	
	float theta_Fd = 0.0;
	// controller command
	float psi_Fc = 0.0;
	float Va_Fc = 0.0; 	
	float theta_Fc = 0.0;
	// wind
	float wl = 0.0;
	float wf = 0.0;
	float wh = 0.0;
	float wal = 0.0;
	float waf = 0.0;
	float wah = 0.0; 
	//////// controller design : step 1 ////////
	if (this->wind_com == "w1"){
		wl = this->w_l_hat;
		wf = this->w_f_hat;
		wh = this->w_h_hat;
		waf = this->wa_f_hat;
		wal = this->wa_l_hat;
		wah = this->wa_h_hat;
		std::cout<<"Comensation on : w1" <<std::endl;
	}		
	else if (this->wind_com == "w2"){
		wl = this->w_l2_hat;
		wf = this->w_f2_hat;
		wh = this->w_h2_hat;
		waf = this->wa_f_hat;
		wal = this->wa_l_hat;
		wah = this->wa_h_hat;
		std::cout<<"Comensation on : w2" <<std::endl;
	}		
	else if (this->wind_com == "none"){
		wl = 0.0;
		wf = 0.0;
		wh = 0.0;
		waf = 0.0;
		wal = 0.0;
		wah = 0.0;
		std::cout<<"Compensation off" <<std::endl;
	}
	else if (this->wind_com == "w"){
		wl = this->w_l;
		wf = this->w_f;
		wh = this->w_h;
		//todo obtain the real wind acceleration 
		waf = 0.0;
		wal = 0.0;
		wah = 0.0;
		std::cout<<"Compensation on : GT" <<std::endl;
	}
	else{
		std::cout<<"No wind com information, compensation off." <<std::endl;
	}
	Va_Fd = sqrt(pow((c1*le+this->leader_angular*fc+wl),2)+pow((c2*fe-this->leader_angular*lc+this->leader_Vg+wf),2));
	psi_Fd = atan2f(-c1*le-this->leader_angular*fc-wl, c2*fe-this->leader_angular*lc+this->leader_Vg+wf)+this->leader_course;
	theta_Fd = asinf((c3*he-this->leader_z_dot+wh)/Va_Fd);

	/*
	if (this->wind_com == "com_on"){
		psi_Fd = atan2f(-c1*le-this->leader_angular*fc-this->w_l_hat, c2*fe-this->leader_angular*lc+this->leader_Vg+this->w_f_hat)+this->leader_course;
		Va_Fd = sqrt(pow((c1*le+this->leader_angular*fc+this->w_l_hat),2)+pow((c2*fe-this->leader_angular*lc+this->leader_Vg+this->w_f_hat),2));
		theta_Fd = asinf((c3*he-this->leader_z_dot+this->w_h_hat)/Va_Fd);
		std::cout<<"wind com on" <<std::endl;
	}
	else if (this->wind_com == "com_off"){
		psi_Fd = atan2f(-c1*le-this->leader_angular*fc, c2*fe-this->leader_angular*lc+this->leader_Vg)+this->leader_course;
		Va_Fd = sqrt(pow((c1*le+this->leader_angular*fc),2)+pow((c2*fe-this->leader_angular*lc+this->leader_Vg),2));
		theta_Fd = asinf((c3*he-this->leader_z_dot)/Va_Fd);
		std::cout<<"wind com off" << std::endl;
	}
	else if (this->wind_com == "com_on_true"){
		psi_Fd = atan2f(-c1*le-this->leader_angular*fc-this->w_l, c2*fe-this->leader_angular*lc+this->leader_Vg+this->w_f)+this->leader_course;
		Va_Fd = sqrt(pow((c1*le+this->leader_angular*fc+this->w_l),2)+pow((c2*fe-this->leader_angular*lc+this->leader_Vg+this->w_f),2));
		theta_Fd = asinf((c3*he-this->leader_z_dot+this->w_h)/Va_Fd);
		std::cout<<"wind com on true" <<std::endl;
	}
	else{
		std::cout<<"No wind com information" <<std::endl;
	}
	*/

	//////// controller design : step 2 ////////
	// assume V_L_dot = theta_L_dot = 0
	// define innerloop inverse time constant
	float alpha = 0.26;
	float beta = 0.1;
	float gamma = 1.3;
	// define state error
	float psi_e = this->yaw - psi_Fd;
	float Va_e = this->Va - Va_Fd;
	float theta_e = this->pitch - theta_Fd;
	// remapping psi_e to [-pi, pi]
	while (psi_e > M_PI){
		psi_e -= 2*M_PI;
	}
	while (psi_e < -M_PI){
		psi_e += 2*M_PI;
	}
	// define formation error dynamics
	float le_dot = this->Va*sinf(this->yaw-this->leader_course) + this->leader_angular*(fe+fc)+wl;
	float fe_dot = this->leader_Vg-this->Va*cosf(this->yaw-this->leader_course) - this->leader_angular*(le+lc)+wf;
	float he_dot = -this->leader_z_dot-this->Va*sinf(this->pitch)+wh;
	// redefine a,b,c, a_dot, b_dot, c_dot
	float a = -c1*le-this->leader_angular*fc-wl;
	float b = c2*fe-this->leader_angular*lc+this->leader_Vg+wf;
	float c = c3*he-this->leader_z_dot+wh;
	float a_dot = -c1*le_dot-wal;
	float b_dot = c2*fe_dot+waf;
	float c_dot = c3*he_dot+wah;
	// define control command
	Va_Fc = 1/alpha*((a_dot*a+b*b_dot)/sqrt(a*a+b*b)-c4*Va_e)+this->Va;
	psi_Fc = 1/beta*((a_dot*b-a*b_dot)/(a*a+b*b)+this->leader_angular-c5*psi_e)+this->yaw;
	theta_Fc = 1/gamma*((c_dot*Va_Fd-c*((a_dot*a+b*b_dot)/sqrt(a*a+b*b)))/(Va_Fd*Va_Fd*sqrt(Va_Fd*Va_Fd-c*c))-c6*theta_e)+this->pitch;
	// controller input constraints
	std::cout<<"Va_Fc raw: "<< Va_Fc << std::endl;
	if (Va_Fc < 12){
		Va_Fc = 12;
	}
	else if (Va_Fc>28){
		Va_Fc = 28;
	}

	// send control command
	VehicleAttitudeSetpoint attitude_set_msg{};
	tf2::Quaternion attitude_quat;
	
	attitude_quat.setRPY(0, theta_Fc, psi_Fc);
	attitude_quat.normalized();
	attitude_set_msg.q_d ={float(attitude_quat.w()),float(attitude_quat.x()),float(attitude_quat.y()),float(attitude_quat.z())}; // w, x, y, z
	attitude_set_msg.thrust_body = {NAN ,0.0, 0.0};
	attitude_set_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	
	this->vehicle_attitude_publisher_->publish(attitude_set_msg);
	this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_CHANGE_SPEED, 0, Va_Fc,-1);
	std::cout<<"Va_Fc : "<< Va_Fc << std::endl;
	std::cout<<"Psi_Fc : "<< psi_Fc << std::endl;
	std::cout<<"theta_Fc : "<< theta_Fc << std::endl;
	
	
	// for plot
	self_msg::msg::Float32MultiArrayStamped cmdInfo{};
	
	cmdInfo.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	cmdInfo.array.data.push_back(Va_Fc);
	cmdInfo.array.data.push_back(Va_Fd);
	cmdInfo.array.data.push_back(this->Va);
	cmdInfo.array.data.push_back(Va_e);
	// cmdInfo.array.data.push_back(Va_Fc-this->Va);
	
	cmdInfo.array.data.push_back(psi_Fc);
	cmdInfo.array.data.push_back(psi_Fd);
	cmdInfo.array.data.push_back(this->yaw);
	cmdInfo.array.data.push_back(psi_e);
	// cmdInfo.array.data.push_back(psi_Fc-this->yaw);

	cmdInfo.array.data.push_back(theta_Fc);
	cmdInfo.array.data.push_back(theta_Fd);
	cmdInfo.array.data.push_back(this->pitch);
	cmdInfo.array.data.push_back(theta_e);
	// cmdInfo.array.data.push_back(theta_Fc-this->pitch);

	this->control_inputs_publisher_->publish(cmdInfo);
	std::cout<<"Va : "<< this->Va << std::endl;
	std::cout<<"Psi : "<< this->yaw << std::endl;
	std::cout<<"theta : "<< this->pitch << std::endl;

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
	msg.position = {0.0, 0.0, this->z0};
	msg.yaw = 0 ; // [-PI:PI]
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
	msg.target_system = this->vehicle_id;
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
