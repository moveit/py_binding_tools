/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * The name of Robert Haschke may not be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Robert Haschke */

#include <pybind11/pybind11.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <py_binding_tools/ros_msg_typecasters.h>

namespace py = pybind11;
namespace
{
struct PubAndSub
{
public:
  explicit PubAndSub(rclcpp::Node::SharedPtr node)
  {
    auto callback = [this](const std_msgs::msg::Int32& message) { value = message.data; };
    pub = node->create_publisher<std_msgs::msg::Int32>("/test", 1);
    sub = node->create_subscription<std_msgs::msg::Int32>("/test", 1, callback);
  }

  void publish(int value)
  {
    std_msgs::msg::Int32 message{};
    message.data = value;
    pub->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub;
  int value{ 0 };
};

PYBIND11_MODULE(py_binding_tools_test, m)
{
  py::class_<PubAndSub>(m, "PubAndSub")
      .def(py::init<rclcpp::Node::SharedPtr>(), py::arg("node"))
      .def("publish", &PubAndSub::publish, py::arg("value"))
      .def_readonly("value", &PubAndSub::value);

  auto increment = [](std_msgs::msg::Int32 msg) {
    msg.data++;
    return msg;
  };
  m.def("inc", increment, "increment data field of Int32 message");

  m.def("incTime", [](rclcpp::Time t) { return t + rclcpp::Duration(std::chrono::nanoseconds(1)); });
}

}  // namespace
