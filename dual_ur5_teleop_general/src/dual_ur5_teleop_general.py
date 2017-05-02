#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
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
# Author: Ioan Sucan

import sys
import rospy
import os
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommandInterpreter, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

interpreter = None

def joy_callback(msg):
    axes = msg.axes
    # Only need to press once to change mode. do a beep.
    if axes[2] < 0:
        interpreter.execute("use left_arm")
        os.system("beep -f 555 -l 460")
    if axes[5] < 0:
        interpreter.execute("use right_arm")
        os.system("beep -f 555 -l 460")


    # Check if LT is pressed
    if axes[2] < 0 and (axes[1] is not 0):
        # Left arm is pressed.
        if axes[1] > 0:
            move_left_arm("forward", axes[1])
        if axes[1] < 0:
            move_left_arm("back", axes[1])
        
    if axes[5] < 0 and (axes[4] is not 0):
        # Right arm is pressed
        if axes[4] > 0:
            move_right_arm("forward", axes[4])
        if axes[4] < 0:
            move_right_arm("back", axes[4])
    if axes[5] < 0 and (axes[0] < 0):
        # Right arm goes left
        global interpreter

def move_right_arm(direction, distance):
    global interpreter
    rospy.loginfo("Moving right arm")
    if direction is "forward":
        interpreter.execute("go forward " + str(distance))
    if direction is "back":
        interpreter.execute("go back " + str(distance))

def move_left_arm(direction, distance):
    global interpreter
    rospy.loginfo("Moving left arm")
    if direction is "forward":
        interpreter.execute("go forward " + str(distance))
    if direction is "back":
        interpreter.execute("go back " + str(distance))

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    rospy.Subscriber("/joy_teleop/joy", Joy, joy_callback, queue_size=1)    

    #scene = PlanningSceneInterface()
    #robot = RobotCommander()
    #rospy.sleep(1)

    #left_arm = robot.left_arm
    #left_arm.go()
    global interpreter
    interpreter = MoveGroupCommandInterpreter()
    interpreter.execute("use left_arm")

    # clean the scene
    # publish a demo scene
   # p = PoseStamped()
   # p.header.frame_id = robot.get_planning_frame()
   # p.pose.position.x = 0.7
   # p.pose.position.y = -0.4
   # p.pose.position.z = 0.85
   # p.pose.orientation.w = 1.0
   # scene.add_box("pole", p, (0.3, 0.1, 1.0))

    #p.pose.position.y = -0.2
    #p.pose.position.z = 0.175
    #scene.add_box("table", p, (0.5, 1.5, 0.35))

    #p.pose.position.x = 0.6
    #p.pose.position.y = -0.7
    #p.pose.position.z = 0.5
    #scene.add_box("part", p, (0.15, 0.1, 0.3))


    # pick an object
    #robot.right_arm.pick("part")

    rospy.spin()

