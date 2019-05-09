#!/usr/bin/env python
import time
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from wifimap import WifiMap
import json
import subprocess
import pandas as pd

def scan():
    try:
        out = subprocess.check_output(['/home/turtlebot/catkin_ws/src/wifi_heatmap/scripts/scan_aps', 'wlp2s0'])
    except subprocess.CalledProcessError:
        return pd.Series([])
    meas = out.encode('utf-8').strip().split('\n')
    meas = json.loads('['  + (', '.join(meas)) + ']')
    meas = {m['MAC'] : m['strength'] for m in meas}
    print('dict', meas)
    print('series', pd.Series(meas))
    return pd.Series(meas)

def publish_wifi_pose():
    publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.init_node('publish_wifi_pose', anonymous=True)
    rate = rospy.Rate(1)
    wifi_pose = PoseWithCovarianceStamped()
    wifi_pose.header.frame_id = "map"


    wmap = WifiMap('../data/dis.json')
    meas = scan()
    print(meas)
    while meas.size <= 1:
        time.sleep(2)
	meas = scan()
    x, y, z, w = wmap.probable_location(meas)

    wifi_pose.pose.pose.position.x = x
    wifi_pose.pose.pose.position.y = y
    wifi_pose.pose.pose.position.z = 0.0

    wifi_pose.pose.pose.orientation.x = 0.0
    wifi_pose.pose.pose.orientation.y = 0.0
    wifi_pose.pose.pose.orientation.z = z
    wifi_pose.pose.pose.orientation.w = w

    # Using covariance from rviz default 2D pose estimate
    wifi_pose.pose.covariance[0] = 10 # X deviation
    wifi_pose.pose.covariance[7] = 10 # Y deviation
    wifi_pose.pose.covariance[35] = 6 # YAW deviation

    if not rospy.is_shutdown():  
        rate.sleep()
        rospy.loginfo(wifi_pose)
        publisher.publish(wifi_pose)

if __name__ == '__main__':
    try:
        publish_wifi_pose()
    except rospy.ROSInterruptException:
        pass
