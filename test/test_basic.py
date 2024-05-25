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
