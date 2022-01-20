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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"

class PongNode : public rclcpp::Node
{
public:
  explicit PongNode(rclcpp::NodeOptions options)
  : Node("pong", options)
  {
    sub_ = this->create_subscription<std_msgs::msg::UInt32>(
      "ping",
      rclcpp::QoS(10),
      std::bind(&PongNode::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::UInt32>(
      "pong",
      rclcpp::QoS(10));
  }

private:
  void callback(const std_msgs::msg::UInt32::SharedPtr msg)
  {
    uint32_t count = msg->data;
    if (count > 0) {
      count--;
    }
    publish(count);
  }

  void publish(const uint32_t count)
  {
    auto msg = std::make_shared<std_msgs::msg::UInt32>();
    msg->data = count;
    RCLCPP_INFO(this->get_logger(), "<- %d", count);
    pub_->publish(*msg);
  }

  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto pong_node = std::make_shared<PongNode>(rclcpp::NodeOptions());
  exec.add_node(pong_node);

  printf("spinning\n");
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
