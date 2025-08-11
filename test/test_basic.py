#####################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2024, Bielefeld University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * The name of Robert Haschke may not be used to endorse or promote
#     products derived from this software without specific prior
#     written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#####################################################################

# Author: Robert Haschke

import pytest
import sys
import time

import rospy
from py_binding_tools import roscpp_init, roscpp_shutdown
from py_binding_tools_test import PubAndSub, inc

from std_msgs.msg import Int32, Bool


@pytest.fixture
def ros_fixture():
    roscpp_init()
    rospy.init_node("test_node")
    yield


@pytest.fixture
def pub_and_sub(ros_fixture):
    return PubAndSub()


def test_multiple_init():
    roscpp_init()
    roscpp_init()
    roscpp_shutdown()
    roscpp_shutdown()

    roscpp_shutdown()  # prints an error


def test_cpp_to_python(pub_and_sub):
    value = 0

    def on_sub(message):
        nonlocal value
        value = message.data

    rospy.Subscriber("/test", Int32, on_sub, queue_size=1)
    time.sleep(0.1)  # wait for connection to be established
    pub_and_sub.publish(42)
    time.sleep(0.1)
    assert value == 42


def test_python_to_cpp(pub_and_sub):
    pub = rospy.Publisher("/test", Int32, queue_size=1)
    time.sleep(0.4)  # wait for connection to be established
    pub.publish(Int32(data=4711))
    time.sleep(0.1)
    assert pub_and_sub.value == 4711


def test_msg_roundtrip():
    result = inc(Int32(data=42))
    assert result.data == 43


def test_msg_conversion_failure():
    with pytest.raises(TypeError):
        inc(Bool(data=False))
    with pytest.raises(TypeError):
        inc(42)


if __name__ == "__main__":
    sys.exit(pytest.main(sys.argv))
