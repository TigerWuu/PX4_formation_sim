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

#include <px4_msgs/msg/vehicle_command.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.h> //WTF?
#include <std_msgs/msg/float32_multi_array.h>
#include <stdint.h>

#include "self_msg/msg/float32_multi_array_stamped.hpp"

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class VehicleCommandControl : public rclcpp::Node
{
public:
	VehicleCommandControl() : Node("vehicle_command")
	{
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		
		auto timer_callback = [this]() -> void {
			this->Va = 12;
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_CHANGE_SPEED, 0, this->Va ,-1);
			std::cout <<"changing speed : "<< this->Va << std::endl;
			// Arm the vehicle
			// this->arm();

		};
		timer_ = this->create_wall_timer(500ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3= 0.0, float param4= 0.0, float param5= 0.0, float param6= 0.0, float param7= 0.0);
	unsigned int Va;
};

/**
 * @brief Send a command to Arm the vehicle
 */
void VehicleCommandControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void VehicleCommandControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void VehicleCommandControl::publish_vehicle_command(uint16_t command, float param1 , float param2, float param3, float param4, float param5 , float param6 , float param7)
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
	std::cout << "Starting Vehicle command testing node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VehicleCommandControl>());

	rclcpp::shutdown();
	return 0;
}
