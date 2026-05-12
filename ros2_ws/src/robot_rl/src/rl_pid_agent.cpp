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
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <algorithm>
#include <random>

class RLPIDAgent : public rclcpp::Node
{
public:
    RLPIDAgent()
    : Node("robot_rl_pid_agent")
    {
        pitch_ = 0.0;
        pitch_rate_ = 0.0;

        kp_ = 0.0;
        ki_ = 0.0;
        kd_ = 0.0;

        gains_initialized_ = false;

        noise_scale_ = 0.2;

        pitch_sub_ = create_subscription<std_msgs::msg::Float64>("/state/pitch", 10, std::bind(&RLPIDAgent::pitchCallback, this, std::placeholders::_1));

        rate_sub_ = create_subscription<std_msgs::msg::Float64>("/state/pitch_rate", 10, std::bind(&RLPIDAgent::rateCallback, this, std::placeholders::_1));

        gain_state_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>("/current_pid_gains", 10, std::bind(&RLPIDAgent::gainStateCallback, this, std::placeholders::_1));

        gain_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/pid_gains", 10);

        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&RLPIDAgent::step, this));

        RCLCPP_INFO(get_logger(), "robot_rl (C++) PID tuner started");
    }

private:
    void pitchCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        pitch_ = msg->data;
    }

    void rateCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        pitch_rate_ = msg->data;
    }

    void gainStateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 3) return;

        if (!gains_initialized_)
        {
            kp_ = msg->data[0];
            ki_ = msg->data[1];
            kd_ = msg->data[2];

            gains_initialized_ = true;

            RCLCPP_INFO(get_logger(), "Initial PID received: kp=%.3f ki=%.3f kd=%.3f", kp_, ki_, kd_);
        }
    }

    double reward()
    {
        return - (std::abs(pitch_) + 0.1 * std::abs(pitch_rate_));
    }

    void step()
    {
        if (!gains_initialized_)
            return;

        double r = reward();

        static std::default_random_engine gen;
        static std::normal_distribution<double> dist(0.0, 1.0);

        double noise = dist(gen);

        double scaled_reward = std::tanh(r);

        kp_ += noise_scale_ * noise * scaled_reward;
        kd_ += noise_scale_ * noise * scaled_reward * 0.5;

        kp_ = std::clamp(kp_, 0.0, 2000.0);
        kd_ = std::clamp(kd_, 0.0, 500.0);
        ki_ = 0.0;

        std_msgs::msg::Float64MultiArray msg;
        msg.data = {kp_, ki_, kd_};

        gain_pub_->publish(msg);

        RCLCPP_INFO(get_logger(), "pitch=%.3f rate=%.3f reward=%.3f kp=%.2f ki=%.2f kd=%.2f", pitch_, pitch_rate_, r, kp_, ki_, kd_);
    }

    double pitch_;
    double pitch_rate_;

    double kp_;
    double ki_;
    double kd_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rate_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gain_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gain_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double noise_scale_;

    bool gains_initialized_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RLPIDAgent>());
    rclcpp::shutdown();
    return 0;
}