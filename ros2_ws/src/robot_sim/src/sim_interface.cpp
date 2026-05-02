/*
 * Copyright (c) 2026, Mezael Docoy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
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
 */
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

class SimInterface : public rclcpp::Node
{
public:

    SimInterface() : Node("sim_interface")
    {
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&SimInterface::jointCallback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>( "/imu/data", 10, std::bind(&SimInterface::imuCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Sim Interface Started");
    }

private:

    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->name.size() < 2 || msg->velocity.size() < 2)
            return;

        double left_vel = 0.0;
        double right_vel = 0.0;

        bool left_found = false;
        bool right_found = false;

        for (size_t i = 0; i < msg->name.size(); i++)
        {
            if (msg->name[i] == "left_wheel_joint")
            {
                left_vel = msg->velocity[i];
                left_found = true;
            }

            if (msg->name[i] == "right_wheel_joint")
            {
                right_vel = msg->velocity[i];
                right_found = true;
            }
        }

        if (!left_found || !right_found)
            return;

        double wheel_radius = 0.05;
        double wheel_base = 0.30;

        double linear_velocity = wheel_radius * (left_vel + right_vel) / 2.0;

        double angular_velocity = wheel_radius * (right_vel - left_vel) / wheel_base;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Vx=%.3f | Wz=%.3f", linear_velocity, angular_velocity);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        latest_imu_ = *msg;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    sensor_msgs::msg::Imu latest_imu_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimInterface>());
    rclcpp::shutdown();
    return 0;
}