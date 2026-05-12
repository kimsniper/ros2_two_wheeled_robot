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
#include <cstdlib>

class RLEpisodeManager : public rclcpp::Node
{
public:
    RLEpisodeManager() : Node("rl_episode_manager")
    {
        pitch_sub_ = create_subscription<std_msgs::msg::Float64>("/state/pitch", 10, std::bind(&RLEpisodeManager::callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&RLEpisodeManager::checkEpisode, this));

        resetting_ = false;

        RCLCPP_INFO(get_logger(), "RL Episode Manager started");
    }

private:

    void callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        pitch_ = msg->data;
    }

    void checkEpisode()
    {
        if (resetting_)
            return;

        if (std::abs(pitch_) > 1.2)
        {
            resetting_ = true;

            RCLCPP_WARN(get_logger(), "Fall detected, resetting simulation");

            std::string cmd =
                "ign service -s /world/empty/set_pose "
                "--reqtype ignition.msgs.Pose "
                "--reptype ignition.msgs.Boolean "
                "--timeout 2000 "
                "--req 'name: \"two_wheel_robot\" position { x: 0 y: 0 z: 0.3 }'";

            std::system(cmd.c_str());

            rclcpp::sleep_for(std::chrono::milliseconds(3000));

            resetting_ = false;
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double pitch_ = 0.0;

    bool resetting_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RLEpisodeManager>());
    rclcpp::shutdown();
    return 0;
}