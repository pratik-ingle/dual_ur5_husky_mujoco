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
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommandInterpreter, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from robotiq_s_model_articulated_msgs.msg import SModelRobotOutput

interpreter = None
left_arm = False
right_arm = False

gripper_publisher = None
gripper_cmd = None

left_gripper_closed = False
right_gripper_closed = False

def genCommand(char, command):
    """Update the command according to the character entered by the user."""    
        
    if char == 'a':
        command = SModelRobotOutput();
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150

    if char == 'r':
        command = SModelRobotOutput();
        command.rACT = 0

    if char == 'c':
        command.rPRA = 255

    if char == 'o':
        command.rPRA = 0

    if char == 'b':
        command.rMOD = 0
        
    if char == 'p':
        command.rMOD = 1
        
    if char == 'w':
        command.rMOD = 2
        
    if char == 's':
        command.rMOD = 3

    #If the command entered is a int, assign this value to rPRA
    try: 
        command.rPRA = int(char)
        if command.rPRA > 255:
            command.rPRA = 255
        if command.rPRA < 0:
            command.rPRA = 0
    except ValueError:
        pass                    
        
    if char == 'f':
        command.rSPA += 25
        if command.rSPA > 255:
            command.rSPA = 255
            
    if char == 'l':
        command.rSPA -= 25
        if command.rSPA < 0:
            command.rSPA = 0

            
    if char == 'i':
        command.rFRA += 25
        if command.rFRA > 255:
            command.rFRA = 255
            
    if char == 'd':
        command.rFRA -= 25
        if command.rFRA < 0:
            command.rFRA = 0

    return command

def joy_callback(msg):
    axes = msg.axes
    buttons = msg.buttons
    global left_arm
    global right_arm
    global left_gripper_closed
    global right_gripper_closed
    global gripper_cmd
    dx = 0.1
    # Dont do anything if the joy command is set to drive.
    if (buttons[0] > 0) or (buttons[2] > 0):
        return False


    # Changing arm control modes. Only one arm controlled at a time.
    if axes[2] < 0:
        interpreter.execute("use left_arm")
        left_arm = True
        right_arm = False
        rospy.loginfo("MODE: LEFT ARM CONTROL MODE")
        return True
    if axes[5] < 0:
        interpreter.execute("use right_arm")
        right_arm = True
        left_arm = False
        rospy.loginfo("MODE: RIGHT ARM CONTROL MODE")
        return True
    # Left trigger pressed, send gripper close
    if buttons[4]:
        if left_gripper_closed:
            open_gripper = genCommand("o", gripper_cmd)
            gripper_publisher.publish(open_gripper)
            rospy.sleep(2)
            rospy.loginfo("Opening left gripper")
            left_gripper_closed = False
        else:
            close_gripper = genCommand("c", gripper_cmd)
            gripper_publisher.publish(close_gripper)
            rospy.sleep(2)
            left_gripper_closed = True
            rospy.loginfo("Closing left gripper")
        return True

    if buttons[5]:
        if right_gripper_closed:
            rospy.loginfo("Opening right gripper")
        else:
            rospy.loginfo("Closing right gripper")
        return True

    left_joy_up = axes[1] > 0
    left_joy_down = axes[1] < 0
    left_joy_left = axes[0] > 0
    left_joy_right = axes[0] < 0

    right_joy_up = axes[4] > 0
    right_joy_down = axes[4] < 0

    # Check if LT is pressed
    if left_arm:
        # Left arm is pressed.
        if left_joy_up:
            move_arm("forward", dx)
        if left_joy_down:
            move_arm("back", dx)
        if left_joy_left:
            move_arm("left", dx)
        if left_joy_right:
            move_arm("right", dx)
        if right_joy_up:
            move_arm("up", dx)
        if right_joy_down:
            move_arm("down", dx)
    elif right_arm:
        # Right arm is pressed
        if left_joy_up:
            move_arm("forward", dx)
        if left_joy_down:
            move_arm("back", dx)
        if left_joy_left:
            move_arm("left", dx)
        if left_joy_right:
            move_arm("right", dx)
        if right_joy_up:
            move_arm("up", dx)
        if right_joy_down:
            move_arm("down", dx)

def move_arm(direction, distance):
    global interpreter
    rospy.loginfo("Moving right arm " + direction + " " + str(distance))
    interpreter.execute("go " + direction + " " + str(distance))

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

    global gripper_publisher
    global gripper_cmd

    gripper_publisher = rospy.Publisher("/SModelRobotOutput", SModelRobotOutput)
    gripper_cmd = SModelRobotOutput()
    gripper_cmd = genCommand("a", gripper_cmd)

    
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

