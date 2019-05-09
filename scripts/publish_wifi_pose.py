#!/usr/bin/env python
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
        return []
    meas = out.encode('utf-8').strip().split('\n')
    meas = json.loads('['  + (', '.join(meas)) + ']')
    meas = {m['MAC'] : m['strength'] for m in meas}
    return pd.Series(meas)

def publish_wifi_pose():
    publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.init_node('publish_wifi_pose', anonymous=True)
    rate = rospy.Rate(1)
    wifi_pose = PoseWithCovarianceStamped()
    wifi_pose.header.frame_id = "map"


    wmap = WifiMap('../data/dis.json')
    x, y = wmap.probable_location(scan())

    wifi_pose.pose.pose.position.x = x
    wifi_pose.pose.pose.position.y = y
    wifi_pose.pose.pose.position.z = 0.0

    wifi_pose.pose.pose.orientation.x = 0.0
    wifi_pose.pose.pose.orientation.y = 0.0
    wifi_pose.pose.pose.orientation.z = -0.0025
    wifi_pose.pose.pose.orientation.w = 0.9999

    # Using covariance from rviz default 2D pose estimate
    wifi_pose.pose.covariance = [0.26914014135545017, -0.0018790245951834095, 0.0, 0.0, 0.0, 0.0, -0.0018790245951834095, 0.22791356565806353, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06814058383126462]

    while not rospy.is_shutdown():  
        rospy.loginfo(wifi_pose)
        publisher.publish(wifi_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_wifi_pose()
    except rospy.ROSInterruptException:
        pass
