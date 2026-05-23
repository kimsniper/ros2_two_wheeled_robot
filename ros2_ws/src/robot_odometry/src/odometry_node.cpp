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
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

#include <cmath>

class OdometryNode : public rclcpp::Node
{
public:

    OdometryNode() : Node("robot_odometry")
    {
        left_velocity_ = 0.0;
        right_velocity_ = 0.0;

        x_ = 0.0;
        y_ = 0.0;
        yaw_ = 0.0;

        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&OdometryNode::jointStateCallback, this, std::placeholders::_1));

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        velocity_pub_ = create_publisher<std_msgs::msg::Float64>("/base_velocity", 10);

        timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&OdometryNode::update, this));

        last_time_ = now();

        RCLCPP_INFO(get_logger(), "Odometry Node Started");
    }

private:

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); i++)
        {
            if (msg->name[i] == "left_wheel_joint")
            {
                left_velocity_ = msg->velocity[i];
            }

            if (msg->name[i] == "right_wheel_joint")
            {
                right_velocity_ = msg->velocity[i];
            }
        }
    }

    void update()
    {
        auto now_time = now();

        double dt = (now_time - last_time_).seconds();

        if (dt <= 0.0)
        {
            return;
        }

        last_time_ = now_time;

        double wheel_radius = 0.08;

        double wheel_base = 0.43;

        double linear_velocity = (((left_velocity_ * wheel_radius) + (right_velocity_ * wheel_radius)) * 0.5);

        double angular_velocity = (((right_velocity_ * wheel_radius) - (left_velocity_ * wheel_radius)) / wheel_base);

        yaw_ += angular_velocity * dt;

        x_ += linear_velocity * std::cos(yaw_) * dt;

        y_ += linear_velocity * std::sin(yaw_) * dt;

        nav_msgs::msg::Odometry odom;

        odom.header.stamp = now_time;

        odom.header.frame_id = "odom";

        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;

        odom.pose.pose.position.y = y_;

        odom.twist.twist.linear.x = linear_velocity;

        odom.twist.twist.angular.z = angular_velocity;

        odom_pub_->publish(odom);

        std_msgs::msg::Float64 vel_msg;

        vel_msg.data = linear_velocity;

        velocity_pub_->publish(vel_msg);

        // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "v=%.3f x=%.3f y=%.3f yaw=%.3f", linear_velocity, x_, y_, yaw_);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    double left_velocity_;

    double right_velocity_;

    double x_;

    double y_;

    double yaw_;

    rclcpp::Time last_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<OdometryNode>());

    rclcpp::shutdown();

    return 0;
}