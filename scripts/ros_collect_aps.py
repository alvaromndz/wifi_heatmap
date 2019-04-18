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
    cproc = subprocess.run(['./scan_aps', 'wlp2s0'], capture_output=True)
    if cproc.returncode == 0 and cproc.stdout is not None:
        strengths = str(cproc.stdout, 'utf-8')
        pos = tfBuffer.lookup_transform("map", 'base_link', rospy.Time(0), rospy.Duration(1.0))
        print('{"x":%.2f, "y":%.2f, "strengths":%s},' % (pos.transform.translation.x, pos.transform.translation.y, lq))
        sleep(0.5)