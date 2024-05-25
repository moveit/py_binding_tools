#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <py_binding_tools/ros_msg_typecasters.h>

namespace py = pybind11;
namespace
{

struct PubAndSub
{
public:
  explicit PubAndSub()
  {
    ros::NodeHandle nh;
    pub = nh.advertise<std_msgs::Int32>("/test", 1);
    sub = nh.subscribe("/test", 1, &PubAndSub::callback, this);
  }

  void publish(int value)
  {
    std_msgs::Int32 message{};
    message.data = value;
    pub.publish(std::move(message));
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
