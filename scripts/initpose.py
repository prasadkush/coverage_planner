#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

from re import T
import rospy
import time

import json

from geometry_msgs.msg import PoseWithCovarianceStamped, Point


def pose_talker():
    rospy.init_node('pose_talker')
    intial_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=36)
    stop_ = rospy.Publisher('/stop_input', Point, queue_size=36)
    num_of_obs_ = rospy.Publisher('/number/obstacles', Point, queue_size=36)
    pose = PoseWithCovarianceStamped()
    stop_pose = Point()
    num_of_obs = Point()
    rate = rospy.Rate(5) # 10hz
    json_data = []
    with open('Map2.json', 'r') as f: 
        json_data = json.load(f) 
        dest_arr = json_data.get('destination')

    x_arr = []
    y_arr = []

    for i in range(len(dest_arr)):
        dest_arr_ = dest_arr[i]
        x = dest_arr_['latitude']
        y = dest_arr_['longitude']
        x_arr.append(x)
        y_arr.append(y)

    check = True
    f = 0
    while check == True: #check_input == True:
        for i in range(len(x_arr)):
            time.sleep(2)
            pose.pose.pose.position.x = x_arr[i]
            pose.pose.pose.position.y = y_arr[i]    
            intial_pose.publish(pose)
            time.sleep(2)
            if (i == len(x_arr)-1):
                stop_pose.x = 1
                num_of_obs.x = 0
                stop_.publish(stop_pose)
                num_of_obs_.publish(num_of_obs)
                stop_.publish(stop_pose)
                check = False
        #rate.sleep()

if __name__ == '__main__':
    try:
        pose_talker()
    except rospy.ROSInterruptException:
        pass
