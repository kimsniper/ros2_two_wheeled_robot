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
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "robot_control/balance_controller.hpp"
#include <cmath>

class BalanceNode : public rclcpp::Node
{
public:

    BalanceNode() : Node("balance_controller")
    {
        pitch_sub_ = create_subscription<std_msgs::msg::Float64>("/state/pitch", 10, std::bind(&BalanceNode::pitchCallback, this, std::placeholders::_1));

        pitch_rate_sub_ = create_subscription<std_msgs::msg::Float64>("/state/pitch_rate", 10, std::bind(&BalanceNode::pitchRateCallback, this, std::placeholders::_1));

        cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_controller/commands", 10);

        controller_ = std::make_shared<BalanceController>();

        controller_->setGains(declare_parameter("kp", 30.0), declare_parameter("ki", 0.0), declare_parameter("kd", 1.2));

        last_time_ = now();

        timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&BalanceNode::updateLoop, this));

        RCLCPP_INFO(get_logger(), "Balance Controller Started");
    }

private:

    void pitchCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        pitch_ = msg->data;
    }

    void pitchRateCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        pitch_rate_ = msg->data;
    }

    void updateLoop()
    {
        auto now_time = now();
        double dt = (now_time - last_time_).seconds();
        last_time_ = now_time;

        double control = controller_->update(pitch_, pitch_rate_, dt);

        if (control > 10.0)
            control = 10.0;

        if (control < -10.0)
            control = -10.0;

        std_msgs::msg::Float64MultiArray cmd;
        cmd.data.resize(2);
        cmd.data[0] = control;
        cmd.data[1] = control;

        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_rate_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;

    std::shared_ptr<BalanceController> controller_;

    rclcpp::Time last_time_;

    rclcpp::TimerBase::SharedPtr timer_;

    double pitch_ = 0.0;
    double pitch_rate_ = 0.0;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BalanceNode>());
    rclcpp::shutdown();
    return 0;
}
