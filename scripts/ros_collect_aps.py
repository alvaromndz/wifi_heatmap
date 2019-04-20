#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-
"""https://hackaday.io/project/25406-wild-thumper-based-ros-robot/log/157556-generating-a-wi-fi-heatmap"""

import os
import re
import sys
import signal
import rospy
import tf
import tf2_ros
import subprocess
from time import sleep

def kill_script(sig, frame):
    """ close array at end of run """
    print(']')
    sys.exit(0)
signal.signal(signal.SIGINT, kill_script)

rospy.init_node('wifi_strength')

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

print('[')
first = True
while not rospy.is_shutdown():
    sleep(0.5)
    try:
        out = subprocess.check_output(['./src/wifi_heatmap/scripts/scan_aps', 'wlp2s0'])
    except subprocess.CalledProcessError:
        continue

    if not first:
        print(',')
    strengths = '['  + (', '.join(out.encode('utf-8').strip().split('\n'))) + ']'
    pos = tfBuffer.lookup_transform("map", 'base_link', rospy.Time(0), rospy.Duration(1.0))
    sys.stdout.write('{"x":%.2f, "y":%.2f, "strengths":%s}' % (pos.transform.translation.x, pos.transform.translation.y, strengths))
    first = False
