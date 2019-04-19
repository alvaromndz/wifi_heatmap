#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import os
import re
import rospy
import tf
import tf2_ros
import subprocess
from time import sleep

rospy.init_node('wifi_strength')

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
regex_lq = re.compile(r"(\w+),\s*([\w:]+),\s*(\d+)")
while not rospy.is_shutdown():
    sleep(0.5)
    try:
    	out = subprocess.check_output(['./src/wifi_heatmap/scripts/scan_aps', 'wlp2s0'])
    except subprocess.CalledProcessError:
	continue

    strengths = out.encode('utf-8')
    pos = tfBuffer.lookup_transform("map", 'base_link', rospy.Time(0), rospy.Duration(1.0))
    print('{"x":%.2f, "y":%.2f, "strengths":%s},' % (pos.transform.translation.x, pos.transform.translation.y, strengths))
