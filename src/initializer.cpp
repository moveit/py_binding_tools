/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Bielefeld University
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
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#include <memory>
#include <mutex>

#include <py_binding_tools/initializer.h>
#include <ros/init.h>
#include <ros/spinner.h>
#include <ros/this_node.h>

namespace py_binding_tools
{
namespace
{
/// singleton class to initialize ROS C++ (once) from Python
class InitProxy
{
public:
  static void init(const std::string& node_name = "",
                   const std::map<std::string, std::string>& remappings = std::map<std::string, std::string>(),
                   uint32_t options = 0);
  static void reset(bool shutdown);

  ~InitProxy()
  {
    spinner_->stop();
    spinner_.reset();
    if (ros::isInitialized() && !ros::isShuttingDown())
      ros::shutdown();
  }

private:
  InitProxy(std::string node_name, const std::map<std::string, std::string>& remappings, uint32_t options)
  {
    if (node_name.empty())
    {
      node_name = "python_wrapper";
      options |= ros::init_options::AnonymousName;
    }
    usage_ = 0;
    if (!ros::isInitialized())
      ros::init(remappings, node_name, options | ros::init_options::NoSigintHandler);
    else
      ROS_WARN_STREAM("ros::init was already called by other means. node name: " << ros::this_node::getName());
    spinner_ = std::make_unique<ros::AsyncSpinner>(1);
    spinner_->start();
  }

  static std::mutex mutex_;
  static std::unique_ptr<InitProxy> singleton_instance_;
  unsigned int usage_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
};
std::mutex InitProxy::mutex_;
std::unique_ptr<InitProxy> InitProxy::singleton_instance_;
}  // namespace

void InitProxy::init(const std::string& node_name, const std::map<std::string, std::string>& remappings,
                     uint32_t options)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (!singleton_instance_)
    singleton_instance_.reset(new InitProxy(node_name, remappings, options));
  else if (!node_name.empty())
    ROS_WARN("ROS C++ was initialized before with name '%s'.%s", ros::this_node::getName().c_str(),
             remappings.empty() ? "" : " Ignoring additional remappings.");
  ++singleton_instance_->usage_;
}

void InitProxy::reset(bool shutdown)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (!singleton_instance_ || singleton_instance_->usage_ == 0)
    ROS_ERROR("Called roscpp_shutdown() without a matching call to roscpp.init()!");
  else if (--singleton_instance_->usage_ == 0 && shutdown)
  {
    // shutdown silences any ROS logging
    ROS_WARN("It's not recommended to call roscpp_shutdown().");
    singleton_instance_.reset();
  }
}

ROScppInitializer::ROScppInitializer()
{
  InitProxy::init();
}
ROScppInitializer::~ROScppInitializer()
{
  InitProxy::reset(false);
}

void roscpp_init(const std::string& node_name, const std::map<std::string, std::string>& remappings, uint32_t options)
{
  InitProxy::init(node_name, remappings, options);
}

void roscpp_shutdown()
{
  // This might shutdown ROS C++, which will stop ROS logging!
  InitProxy::reset(true);
}

}  // namespace py_binding_tools
