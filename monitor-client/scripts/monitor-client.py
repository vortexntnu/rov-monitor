#!/usr/bin/python
import rospy

from monitor_msgs.msg import DeviceStatus


class MonitorClientNode(object):
    def __init__(self):
        rospy.init_node("monitor_client_node")
        self.pub_device_status = rospy.Publisher('device_status',
                                                 DeviceStatus,
                                                 queue_size=1)


if __name__ == '__main__':
    monitor_client_node = MonitorClientNode()
