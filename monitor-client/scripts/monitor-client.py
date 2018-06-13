#!/usr/bin/python
import rospy
import socket
import os

from monitor_msgs.msg import DeviceStatus


class MonitorClientNode(object):
    def __init__(self):
        rospy.init_node("monitor_client_node")
        self.pub_device_status = rospy.Publisher('device_status',
                                                 DeviceStatus,
                                                 queue_size=1)

        def get_ip():
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("192.168.0.1", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip

        def get_cpu_temp():
            try:
                with open('/sys/class/thermal/thermal_zone0/temp') as temp_file:
                    temp = int(temp_file.read())
                    return temp/1000
            except:
                return -1

        def get_leak_status():
            return -1  # TODO: Check if leak sensor is available, read if it is

        def get_id():
            try:
                with open('/sys/block/mmcblk0/device/cid') as id_file:
                    device_id = id_file.read()
            except:
                device_id = "N/A"
            return device_id

        def get_type():
            try:
                device_type = os.environ["VORTEX_DEVICE_TYPE"]
            except KeyError:
                rospy.roswarn("Failed to get device type")
                device_type = "Unknown"
            return device_type

if __name__ == '__main__':
    monitor_client_node = MonitorClientNode()
