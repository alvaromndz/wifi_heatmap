#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-
"""https://hackaday.io/project/25406-wild-thumper-based-ros-robot/log/157556-generating-a-wi-fi-heatmap"""

import os
import re
import sys
import signal
import argparse
import rospy
import tf
import tf2_ros
import subprocess
from time import sleep

def scan():
    try:
        out = subprocess.check_output(['/home/turtlebot/catkin_ws/src/wifi_heatmap/scripts/scan_aps', 'wlp2s0'])
    except subprocess.CalledProcessError:
        return None
    
    return out.encode('utf-8').strip().split('\n')

def main(f, scans_per):
    rospy.init_node('wifi_strength')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)


    first = True
    while not rospy.is_shutdown():
        raw_input('Press enter to start scan...')

        pos = tfBuffer.lookup_transform("map", 'base_link', rospy.Time(0), rospy.Duration(1.0))
        for i in range(scans_per):
            meas = scan()
            if meas is not None:
                print('scan{}: {} aps'.format(i, len(meas)))
            else:
                print('scan{}: FAILED'.format(i))
            
            strengths = '['  + (', '.join(meas)) + ']'

            if not first:
                f.write(',\n')
            f.write('{"x":%.2f, "y":%.2f, "z":%.2f, "w":%.2f, "aps":%s}' % (pos.transform.translation.x, pos.transform.translation.y, pos.transform.rotation.z, pos.transform.rotation.w, strengths))
            first = False
                
def kill_script(f):
    """ close array at end of run """
    f.write(']\n')
    sys.exit(0)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='collect locations and access point data using ros')
    parser.add_argument('-n', '--scans-per', type=int, default=5,
                            help='Number of scans to take per location')
    parser.add_argument('outfile', help='output filename')

    args = parser.parse_args()
    with open(args.outfile, 'w+') as f:
        f.write('[\n')
        signal.signal(signal.SIGINT, lambda sig, frame: kill_script(f))
        main(f, args.scans_per)
