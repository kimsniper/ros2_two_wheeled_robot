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

#include <unordered_map>
#include <string>
#include <cmath>

class EncoderProcessor : public rclcpp::Node
{
public:

    EncoderProcessor()
    : Node("encoder_processor")
    {
        declare_parameter("ticks_per_revolution", 2048);

        sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&EncoderProcessor::callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Encoder Processor Started");
    }

private:

    void callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->name.empty())
            return;

        double left_rad = 0.0;
        double right_rad = 0.0;

        bool left_found = false;
        bool right_found = false;

        for (size_t i = 0; i < msg->name.size(); i++)
        {
            if (msg->name[i] == "left_wheel_joint")
            {
                left_rad = msg->position[i];
                left_found = true;
            }

            if (msg->name[i] == "right_wheel_joint")
            {
                right_rad = msg->position[i];
                right_found = true;
            }
        }

        if (!left_found || !right_found)
            return;

        int tpr = get_parameter("ticks_per_revolution").as_int();

        int left_ticks = static_cast<int>((left_rad / (2.0 * M_PI)) * tpr);

        int right_ticks = static_cast<int>((right_rad / (2.0 * M_PI)) * tpr);

        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,"ENCODER -> L ticks=%d | R ticks=%d",left_ticks,right_ticks);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EncoderProcessor>());
    rclcpp::shutdown();
    return 0;
}