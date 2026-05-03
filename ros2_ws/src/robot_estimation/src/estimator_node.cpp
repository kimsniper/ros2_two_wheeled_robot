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
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/float64.hpp"

#include "robot_estimation/imu_ekf.hpp"
#include <cmath>

class EstimatorNode : public rclcpp::Node
{
public:

    EstimatorNode() : Node("robot_estimator")
    {
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, std::bind(&EstimatorNode::imuCallback, this, std::placeholders::_1));

        quat_pub_ = create_publisher<geometry_msgs::msg::Quaternion>("/state/imu_quaternion", 10);

        pitch_pub_ = create_publisher<std_msgs::msg::Float64>("/state/pitch", 10);

        pitch_rate_pub_ = create_publisher<std_msgs::msg::Float64>("/state/pitch_rate", 10);

        ekf_ = std::make_shared<ImuEKF>();
        last_time_ = this->now();

        RCLCPP_INFO(get_logger(), "Robot Estimator Started");
    }

private:

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        rclcpp::Time current_time = msg->header.stamp;
        double dt = (current_time - last_time_).seconds();
        
        if (dt <= 0.0 || dt > 0.1) {
            dt = 0.01; // Default to 100Hz if clock is invalid
        }
        last_time_ = current_time;
        ekf_->setDt(static_cast<float>(dt));

        std::array<float,3> gyro = {
            (float)msg->angular_velocity.x,
            (float)msg->angular_velocity.y,
            (float)msg->angular_velocity.z
        };

        std::array<float,3> accel = {
            (float)msg->linear_acceleration.x,
            (float)msg->linear_acceleration.y,
            (float)msg->linear_acceleration.z
        };

        ekf_->predict(gyro);
        ekf_->update(accel);

        auto q = ekf_->getQuaternion();

        geometry_msgs::msg::Quaternion out;
        out.x = q[0];
        out.y = q[1];
        out.z = q[2];
        out.w = q[3];

        quat_pub_->publish(out);

        double pitch = asin(std::max(-1.0, std::min(1.0, 2.0 * (out.w * out.y - out.z * out.x))));
        std_msgs::msg::Float64 p;
        p.data = pitch;
        pitch_pub_->publish(p);

        std_msgs::msg::Float64 pr;
        pr.data = msg->angular_velocity.y;
        pitch_rate_pub_->publish(pr);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr quat_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_rate_pub_;

    std::shared_ptr<ImuEKF> ekf_;
    rclcpp::Time last_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EstimatorNode>());
    rclcpp::shutdown();
    return 0;
}