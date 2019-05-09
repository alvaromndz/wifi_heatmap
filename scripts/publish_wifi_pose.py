#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def publish_wifi_pose():
    publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.init_node('publish_wifi_pose', anonymous=True)
    rate = rospy.Rate(1)
    wifi_pose = PoseWithCovarianceStamped()
    wifi_pose.header.frame_id = "map"

    wifi_pose.pose.pose.position.x = 19.0
    wifi_pose.pose.pose.position.y = 7.8
    wifi_pose.pose.pose.position.z = 0.0

    wifi_pose.pose.pose.orientation.x = 0.0
    wifi_pose.pose.pose.orientation.y = 0.0
    wifi_pose.pose.pose.orientation.z = -0.0025
    wifi_pose.pose.pose.orientation.w = 0.9999

    while not rospy.is_shutdown():  
        rospy.loginfo(wifi_pose)
        publisher.publish(wifi_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_wifi_pose()
    except rospy.ROSInterruptException:
        pass