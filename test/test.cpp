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

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <py_binding_tools/ros_msg_typecasters.h>

namespace py = pybind11;
namespace
{
struct PubAndSub
{
public:
  PubAndSub()
  {
    ros::NodeHandle nh;
    pub = nh.advertise<std_msgs::Int32>("/test", 1);
    sub = nh.subscribe("/test", 1, &PubAndSub::callback, this);
  }

  void publish(int value)
  {
    std_msgs::Int32 message{};
    message.data = value;
    pub.publish(message);
  }

  void callback(const std_msgs::Int32& message)
  {
    value = message.data;
  }

  ros::Publisher pub;
  ros::Subscriber sub;
  int value{ 0 };
};

PYBIND11_MODULE(py_binding_tools_test, m)
{
  py::class_<PubAndSub>(m, "PubAndSub")
      .def(py::init<>())
      .def("publish", &PubAndSub::publish, py::arg("value"))
      .def_readonly("value", &PubAndSub::value);

  auto increment = [](std_msgs::Int32 msg) {
    msg.data++;
    return msg;
  };
  m.def("inc", increment, "increment data field of Int32 message");
}

}  // namespace
