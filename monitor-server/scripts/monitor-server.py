#!/usr/bin/python
import rospy

from monitor_msgs.msg import DeviceList, DeviceStatus


class MonitorServerNode(object):
    def __init__(self):
        rospy.init_node("monitor_server_node")
        self.devices = {}
        self.pub_device_list = rospy.Publisher('device_list',
                                               DeviceList,
                                               queue_size=10)
        self.sub_statuses = rospy.Subscriber('device_status',
                                             DeviceStatus,
                                             self.device_status_cb,
                                             queue_size=10)

    def publish_devices(self):
        device_list_msg = DeviceList()
        for ip in self.devices:
            device_list_msg.devices.append(self.devices[ip])
        device_list_msg.header.stamp = rospy.get_rostime()
        self.pub_device_list.publish(device_list_msg)

    def device_status_cb(self, device):
        self.devices[device.device_id] = device


if __name__ == '__main__':
    monitor_server_node = MonitorServerNode()
    while not rospy.is_shutdown():
        monitor_server_node.publish_devices()
        rospy.Rate(1).sleep()
