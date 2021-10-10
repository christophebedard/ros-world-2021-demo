// Copyright 2021 Christophe Bedard
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"

class PingNode : public rclcpp::Node
{
public:
  explicit PingNode(
    rclcpp::NodeOptions options,
    const uint32_t initial_count,
    const uint32_t period,
    const uint32_t sequences)
  : Node("ping", options),
    initial_count_(initial_count),
    sequences_(sequences)
  {
    RCLCPP_INFO(this->get_logger(), "initial count = %d", initial_count_);
    RCLCPP_INFO(this->get_logger(), "period        = %d", period);
    RCLCPP_INFO(this->get_logger(), "sequences     = %d", sequences_);
    sequences_++;
    sub_ = this->create_subscription<std_msgs::msg::UInt32>(
      "pong",
      10,
      std::bind(&PingNode::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::UInt32>(
      "ping",
      10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period),
      std::bind(&PingNode::timer_callback, this));
  }

private:
  void callback(const std_msgs::msg::UInt32::ConstSharedPtr msg)
  {
    uint32_t count = msg->data;
    if (0 == count) {
      RCLCPP_INFO(this->get_logger(), "end sequence\n");
      if (0 == sequences_) {
        RCLCPP_INFO(this->get_logger(), "shutting down");
        rclcpp::shutdown();
      }
    } else {
      publish(count - 1);
    }
  }

  void timer_callback()
  {
    if (sequences_ > 0) {
      sequences_--;
    }
    RCLCPP_INFO(this->get_logger(), "sequence %d", sequences_);
    publish(initial_count_);
  }

  void publish(const uint32_t count)
  {
    auto msg = std::make_shared<std_msgs::msg::UInt32>();
    msg->data = count;
    RCLCPP_INFO(this->get_logger(), "-> %d", count);
    pub_->publish(*msg);
  }

  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  const uint32_t initial_count_;
  uint32_t sequences_;
};

int main(int argc, char * argv[])
{
  uint32_t initial_count = 0;
  uint32_t period = 0;
  uint32_t sequences = 0;
  for (int i = 0; i < argc; ++i) {
    if (std::string(argv[i]) == "count" && i + 1 < argc) {
      initial_count = std::stoul(argv[i + 1]);
    }
    if (std::string(argv[i]) == "period" && i + 1 < argc) {
      period = std::stoul(argv[i + 1]);
    }
    if (std::string(argv[i]) == "sequences" && i + 1 < argc) {
      sequences = std::stoul(argv[i + 1]);
    }
  }

  if (0 == initial_count || 0 == period || 0 == sequences) {
    printf(
      "bad or missing values: initial_count=%d, period=%d, sequences=%d\n",
      initial_count,
      period,
      sequences);
    return 1;
  }

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto ping_node = std::make_shared<PingNode>(
    rclcpp::NodeOptions(),
    initial_count,
    period,
    sequences);
  exec.add_node(ping_node);

  printf("spinning\n");
  exec.spin();

  // Will actually be called inside the node's callback
  rclcpp::shutdown();
  return 0;
}
