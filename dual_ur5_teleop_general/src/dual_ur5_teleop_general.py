#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Clearpath Robotics, Inc.
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
#  * Neither the name of Clearpath Robotics, Inc. nor the names of its
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
# Author: Devon Ash

import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommandInterpreter, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from robotiq_s_model_articulated_msgs.msg import SModelRobotOutput
from trajectory_msgs.msg import *
from control_msgs.msg import *
import actionlib
import tf

interpreter = None
left_arm = False
right_arm = False
gripper_mode = False
ptu_mode = False

r_gripper_publisher = None
l_gripper_publisher = None
gripper_cmd = None

left_gripper_closed = False
right_gripper_closed = False

left_gripper_rotation = 0
right_gripper_rotation = 0

husky_ptu_pan_position = 0
husky_ptu_tilt_position = 0

left_arm_client = None
right_arm_client = None

ptu_cmd_publisher = None

LEFT_JOINT_NAMES = ['l_ur5_arm_shoulder_pan_joint', 'l_ur5_arm_shoulder_lift_joint', 'l_ur5_arm_elbow_joint', 'l_ur5_arm_wrist_1_joint', 'l_ur5_arm_wrist_2_joint', 'l_ur5_arm_wrist_3_joint']
RIGHT_JOINT_NAMES = ['r_ur5_arm_shoulder_pan_joint', 'r_ur5_arm_shoulder_lift_joint', 'r_ur5_arm_elbow_joint', 'r_ur5_arm_wrist_1_joint', 'r_ur5_arm_wrist_2_joint', 'r_ur5_arm_wrist_3_joint']
Q1 = [1.57, -1.57, 0, -1.57, 0, 0]
Q2 = [-1.57, -1.57, 0, -1.57, 0, 0]
Q3 = [-1.57, -0.2967, -2.79, -1.57, -0.5236, 0]

RQ1 = [0, -1.57, 0, -1.5708, 0, 0]
RQ2 = [1.57, -1.57, 0, -1.57, 0, 0]
RQ3 = [1.57, -2.8449, 2.7925, -1.57, 0.5236, 0]

# Definitions for predefined locations
GRAB_OUT = 0
GRAB_IN = 1
GRAB_ABOVE = 2
GRAB_BELOW = 3
GRAB_FRONT = 4
CARRY = 5
DROP_OFF = 6
FRONTAL_GRAB = 7

LEFT_ARM = 0
RIGHT_ARM = 1

left_arm_joint_settings = [
[-1.02, -0.035, -0.55, -1.453, -2.22, -1.481], 
[],
[],
[],
[],
[],
[] ]
left_arm_pose_settings = [
[0.899, 0.371, 0.0066, 0.268, 0.265, 0.6686, 0.6405] ]
left_arm_rpy_settings = [
[0.7739, -0.019, 1.605] ]

right_arm_joint_settings = [[]]
right_arm_pose_settings = [[]]
right_arm_rpy_settings = [[]]

CONTROL_MODE = 0
LEFT_ARM_CONTROL = 1
RIGHT_ARM_CONTROL = 2
GRIPPER_CONTROL = 3

CURRENT_JOINT_CONTROL = None
NUMBER_OF_JOINTS = 0 
JOINT_LIMIT_RADIANS = 2
INDIVIDUAL_JOINT_CONTROL = 4

from subprocess import Popen, PIPE, STDOUT

# Ignores anything with a greater then 3 hz frequency
def update_joint_selector(next=False, previous=False):
    global CURRENT_JOINT_CONTROL
    incr = None
    if next:
        incr = 1
        rospy.loginfo("Going to next joint")
    elif previous:
        incr = -1
        rospy.loginfo("Going to previous joint")

    if CURRENT_JOINT_CONTROL is None:
        CURRENT_JOINT_CONTROL = 0
    elif CURRENT_JOINT_CONTROL >= 0:
        CURRENT_JOINT_CONTROL = CURRENT_JOINT_CONTROL + incr    
        if CURRENT_JOINT_CONTROL >= 6:
            CURRENT_JOINT_CONTROL = None
        if CURRENT_JOINT_CONTROL <= 0:
            CURRENT_JOINT_CONTROL = None
        
    vibrate_controller()
    rospy.loginfo("Currently controlling joint: " + str(CURRENT_JOINT_CONTROL))

# Needs to be locking
lock = False
def vibrate_controller():
    global lock
    if lock is False:
        lock = True
        process = Popen(["fftest", "/dev/input/event9"], stdout=PIPE, stdin=PIPE, stderr=STDOUT)
        process.stdin.write("4\n0\n")
        rospy.sleep(2.0)
        process.stdin.write("-1\n")
        lock = False

# Returns False if it cannot move it past the joint range.
# Returns True if it can actuate it via MoveIt! (e.g its within the range)
def move_selected_joint(direction=1):
    # Take current joint position from current mode/arm position by listening to group.
    # Increment that joint position. If its above the limits, do not move it. Instead, vibrate the controller.
    global JOINT_LIMIT_RADIANS
    global interpreter
    increment = 0.2 * direction
    group = interpreter.get_active_group()
    global left_arm_client
    global right_arm_client
    global LEFT_JOINT_NAMES
    global RIGHT_JOINT_NAMES

    client = None
    JOINTS = None

    if CONTROL_MODE is LEFT_ARM_CONTROL:
        client = left_arm_client
        JOINTS = LEFT_JOINT_NAMES
    elif CONTROL_MODE is RIGHT_ARM_CONTROL:
        client = right_arm_client
        JOINTS = RIGHT_JOINT_NAMES
    else:
        rospy.loginfo("You must select an arm to control before commanding the joint modes")
        return False

    Q1 = group.get_current_joint_values()

    current_position = Q1[CURRENT_JOINT_CONTROL] 
    updated_position = current_position + increment
    Q1[CURRENT_JOINT_CONTROL] = updated_position

    if updated_position > 2.0:
        updated_position = 2.0
        vibrate_controller()
    if updated_position < -2.0:
        updated_position = -2.0
        vibrate_controller()

    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINTS
    g.trajectory.points = [
        JointTrajectoryPoint(positions=group.get_current_joint_values(), velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(1.3)) ]
    client.send_goal(g)
    rospy.loginfo("Moving joint " + group.get_joints()[CURRENT_JOINT_CONTROL] + " to new position at " + str(updated_position))
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_ptu(increment=0.2, tilt=False, pan=False, direction=1):
    global ptu_cmd_publisher
    speed = 0.1
    increment = increment * direction
    rospy.loginfo("Direction: " + str(increment))
    joint_state_msg = JointState()
    joint_state_msg.name = ['husky_ptu_pan', 'husky_ptu_tilt']
    joint_state_msg.header.frame_id = 'husky_ptu'
    # current position
    rospy.loginfo("Current position pan: " + str(husky_ptu_pan_position) + " tilt: " + str(husky_ptu_tilt_position))
    joint_state_msg.position = [husky_ptu_pan_position, husky_ptu_tilt_position]
    joint_state_msg.effort = [0, 0]
    joint_state_msg.header.seq = 0
    # increment it 
    if pan:
        joint_state_msg.position[0] = joint_state_msg.position[0] + increment
    if tilt:
        joint_state_msg.position[1] = joint_state_msg.position[1] + increment

    joint_state_msg.velocity = [speed, speed]
    ptu_cmd_publisher.publish(joint_state_msg)

        
def go_to_predefined(conf):
    # CONF 0 GRAB OUT
    # CONF 1 GRAB IN
    # CONF 2 GRAB ABOVE
    # CONF 3 GRAB BELOW
    # CONF 4 GRAB FRONT
    # CONF 5 CARRY
    # CONF 6 DROP_OFF
    # CONF 7 FRONTAL GRAB

    global GRAB_OUT
    global GRAB_IN
    global GRAB_ABOVE
    global GRAB_BELOW
    global GRAB_FRONT
    global CARRY
    global DROP_OFF
    global FRONTAL_GRAB
    global interpreter
    global LEFT_ARM
    global RIGHT_ARM
    global left_arm_joint_settings
    global left_arm_pose_settings
    global left_arm_rpy_settings
    global right_arm_joint_settings
    global right_arm_pose_settings
    global right_arm_rpy_settings
    global LEFT_ARM_CONTROL
    global RIGHT_ARM_CONTROL
    global CONTROL_MODE
  
    joint_settings = None
    pose_settings = None
    rpy_settings = None

    group = interpreter.get_active_group()
    joints = group.get_joints()
    arm = joints[0][0] 
    if arm is 'l':
        joint_settings = left_arm_joint_settings
        rpy_settings = left_arm_rpy_settings
        pose_settings = left_arm_pose_settings
    elif arm is 'r':
        joint_settings = right_arm_joint_settings
        rpy_settings = right_arm_rpy_settings
        pose_settings = right_arm_pose_settings 

    group.set_joint_value_target(joint_settings[conf])
    group.set_rpy_target(rpy_settings[conf])
    group.set_pose_target(pose_settings[conf])

    group.go()

def move_home():
    global interpreter
    group = interpreter.get_active_group()
    global left_arm_client
    global right_arm_client
    global CONTROL_MODE
    global LEFT_ARM_CONTROL
    global RIGHT_ARM_CONTROL
    JOINTS = None
    client = None

    global Q1
    global Q2
    global Q3
    global RQ1
    global RQ2
    global RQ3

    HQ1 = None
    HQ2 = None
    HQ3 = None
   
    if CONTROL_MODE is LEFT_ARM_CONTROL: 
        client = left_arm_client
        JOINTS = LEFT_JOINT_NAMES
        HQ1 = Q1
        HQ2 = Q2
        HQ3 = Q3
    elif CONTROL_MODE is RIGHT_ARM_CONTROL:
        client = right_arm_client
        JOINTS = RIGHT_JOINT_NAMES
        HQ1 = RQ1
        HQ2 = RQ2
        HQ3 = RQ3

    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINTS 
    g.trajectory.points = [
        JointTrajectoryPoint(positions=group.get_current_joint_values(), velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
        JointTrajectoryPoint(positions=HQ1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)), 
        JointTrajectoryPoint(positions=HQ2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=HQ3, velocities=[0]*6, time_from_start=rospy.Duration(6.0))]
    client.send_goal(g)
    rospy.loginfo("Moved arm to home position")
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise


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

def rotate_gripper(direction, which_gripper):
    global left_gripper_rotation
    global right_gripper_rotation

    rotation_step = 0.205
    
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    
    LEFT_ARM_JOINT_NAMES = ['l_ur5_arm_shoulder_pan_joint', 'l_ur5_arm_shoulder_lift_joint', 'l_ur5_arm_elbow_joint', 'l_ur5_arm_wrist_1_joint', 'l_ur5_arm_wrist_2_joint', 'l_ur5_arm_wrist_3_joint']
    RIGHT_ARM_JOINT_NAMES = ['r_ur5_arm_shoulder_pan_joint', 'r_ur5_arm_shoulder_lift_joint', 'r_ur5_arm_elbow_joint', 'r_ur5_arm_wrist_1_joint', 'r_ur5_arm_wrist_2_joint', 'r_ur5_arm_wrist_3_joint']
    
    if which_gripper is "left":
        g.trajectory.joint_names = LEFT_ARM_JOINT_NAMES
    if which_gripper is "right":
        g.trajectory.joint_names = RIGHT_ARM_JOINT_NAMES
    

    global interpreter
    current_group = interpreter.get_active_group()
    joints = current_group.get_current_joint_values()
    old_joints = joints
    rospy.loginfo(joints)
    global left_arm_client
    global right_arm_client

    # use the direction to move the joint value up or down. it's in radians. so should always figure itself out? or maybe needed to be bounded?
    if direction is "rotate_left":
        # Decrease the joint position in radians.
        val = joints[5]
        val += abs(rotation_step)
        joints[5] = val    
    if direction is "rotate_right":
        val = joints[5]
        val = val - abs(rotation_step)
        joints[5] = val

    current_group.set_joint_value_target(joints)
    current_group.go()

    return True

def joy_callback(msg):
    axes = msg.axes
    buttons = msg.buttons
    global left_arm
    global right_arm
    global left_gripper_closed
    global right_gripper_closed
    global gripper_cmd
    global gripper_mode
    global ptu_mode
    global LEFT_ARM_CONTROL
    global CONTROL_MODE
    global RIGHT_ARM_CONTROL
    global GRIPPER_CONTROL

    dpad_right = axes[6] < 0
    dpad_left = axes[6] > 0
    dpad_down = axes[7] < 0
    dpad_up = axes[7] > 0

    left_joy_up = axes[1] > 0
    left_joy_down = axes[1] < 0
    left_joy_left = axes[0] > 0
    left_joy_right = axes[0] < 0

    left_joy_directions = [left_joy_up, left_joy_down, left_joy_left, left_joy_right]

    right_joy_up = axes[4] > 0
    right_joy_down = axes[4] < 0
    right_joy_left = axes[3] > 0
    right_joy_right = axes[3] < 0

    right_joystick_pressed = (buttons[10] == 1)
    left_joy_pressed = (buttons[9] == 1)

    dx = 0.05
    fast_driving = (buttons[2] > 0 and (left_joy_up or left_joy_down or left_joy_left or left_joy_right))
    slow_driving = (buttons[0] > 0 and (left_joy_up or left_joy_down or left_joy_left or left_joy_right))
    is_driving = slow_driving or fast_driving
    # Dont do anything if the joy command is set to drive.
    if is_driving:
        return False

    if buttons[6]:
        # stow arms
        move_home()
        return

    a = msg.header.seq % 7
    if buttons[3] and (a == 0):
        update_joint_selector(next=True)
    
    if buttons[2] and (a == 0):
        update_joint_selector(previous=True)

    if dpad_left:
        rospy.loginfo("Panning left")
        move_ptu(pan=True, direction=1)
    if dpad_right:
        rospy.loginfo("Panning right")
        move_ptu(pan=True, direction=-1)
    if dpad_down:
        rospy.loginfo("Tilting down")
        move_ptu(tilt=True, direction=-1)
    if dpad_up:
        rospy.loginfo("Tilting up")
        move_ptu(tilt=True, direction=1)

    # Changing arm control modes. Only one arm controlled at a time.
    if axes[2] < 0:
        interpreter.execute("use left_arm")
        rospy.loginfo("MODE: LEFT ARM CONTROL MODE")
        CONTROL_MODE = LEFT_ARM_CONTROL
        vibrate_controller()
        return True
    if axes[5] < 0:
        interpreter.execute("use right_arm")
        CONTROL_MODE = RIGHT_ARM_CONTROL
        rospy.loginfo("MODE: RIGHT ARM CONTROL MODE")
        vibrate_controller() 
        return True
    # Left trigger pressed, send gripper close
    if buttons[4]:
        if left_gripper_closed:
            open_gripper = genCommand("o", gripper_cmd)
            l_gripper_publisher.publish(open_gripper)
            rospy.sleep(2)
            rospy.loginfo("Opening left gripper")
            left_gripper_closed = False
        else:
            close_gripper = genCommand("c", gripper_cmd)
            l_gripper_publisher.publish(close_gripper)
            rospy.sleep(2)
            left_gripper_closed = True
            rospy.loginfo("Closing left gripper")
        return True
    if buttons[5]:
        if right_gripper_closed:
            open_gripper = genCommand("o", gripper_cmd)
            r_gripper_publisher.publish(open_gripper)
            rospy.sleep(0.5)
            right_gripper_closed = False
            rospy.loginfo("Opening right gripper")
        else:
            close_gripper = genCommand("c", gripper_cmd)
            r_gripper_publisher.publish(close_gripper)
            rospy.sleep(0.5)
            right_gripper_closed = True
            rospy.loginfo("Closing right gripper")
        return True
    if buttons[1]:
        CONTROL_MODE = GRIPPER_CONTROL
        if not gripper_mode:
            gripper_mode = True
            vibrate_controller()
        else:
            gripper_mode = False

    global GRAB_OUT
    global GRAB_IN
    global GRAB_FORWARD
    global GRAB_ABOVE
    global GRAB_BELOW
    global INDIVIDUAL_JOINT_CONTROL
    global CURRENT_JOINT_CONTROL    

    # Check if LT is pressed
    if CONTROL_MODE is LEFT_ARM_CONTROL and (CURRENT_JOINT_CONTROL is None):
        # Left arm is pressed.
        if left_joy_up:
            move_arm("forward", dx)
        elif left_joy_down:
            move_arm("back", dx)
        elif left_joy_left:
            move_arm("left", dx)
        elif left_joy_right:
            move_arm("right", dx)
        elif right_joy_up:
            move_arm("up", dx)
        elif right_joy_down:
            move_arm("down", dx)
    elif CONTROL_MODE is RIGHT_ARM_CONTROL and (CURRENT_JOINT_CONTROL is None):
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
    elif CONTROL_MODE is GRIPPER_CONTROL and (CURRENT_JOINT_CONTROL is None):
        arm = ""
        if left_arm:
            arm = "left"
        elif right_arm:
            arm = "right"
        if left_joy_left:
            go_to_predefined(GRAB_OUT)
        elif left_joy_right:
            go_to_predefined(GRAB_IN)
        elif left_joy_up:
            go_to_predefined(GRAB_ABOVE)
        elif left_joy_down:
            go_to_predefined(GRAB_BELOW)
        elif left_joy_pressed:
            go_to_predefined(GRAB_FORWARD)
        elif right_joy_left:
            rotate_gripper("rotate_left", arm)
        elif right_joy_right:
            rotate_gripper("rotate_right", arm)
    elif CURRENT_JOINT_CONTROL >= 0:
        if right_joy_left:
            move_selected_joint(direction=1)
        if right_joy_right:
            move_selected_joint(direction=-1)        
    elif ptu_mode:
        return True

def jointstate_callback(msg):
    # Get current js for ptu
    global husky_ptu_tilt_position
    global husky_ptu_pan_position
    joints = msg.name
    husky_ptu_pan_position = msg.position[0] 
    husky_ptu_tilt_position = msg.position[1]
    
def move_arm(direction, distance):
    global interpreter
    rospy.loginfo("Moving right arm " + direction + " " + str(distance))
    interpreter.execute("go " + direction + " " + str(distance))

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    rospy.Subscriber("/joy_teleop/joy", Joy, joy_callback, queue_size=1)    

    global interpreter
    interpreter = MoveGroupCommandInterpreter()
    interpreter.execute("use left_arm")

    global r_gripper_publisher
    global l_gripper_publisher
    global gripper_cmd

    global left_arm_client
    global right_arm_client
   
    r_gripper_publisher = rospy.Publisher("/r_gripper/SModelRobotOutput", SModelRobotOutput)
    l_gripper_publisher = rospy.Publisher("/l_gripper/SModelRobotOutput", SModelRobotOutput)

    ptu_cmd_publisher = rospy.Publisher("/ptu/cmd", JointState)
    joint_state_subscriber = rospy.Subscriber("/joint_states_ptu", JointState, jointstate_callback, queue_size=1)
    
    gripper_cmd = SModelRobotOutput()
    gripper_cmd = genCommand("a", gripper_cmd)

    left_arm_client = actionlib.SimpleActionClient('/l_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    right_arm_client = actionlib.SimpleActionClient('/r_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for left arm action server")
    left_arm_client.wait_for_server()
    rospy.loginfo("Waiting for right arm action server")
    right_arm_client.wait_for_server()
    rospy.loginfo("Connected to arm action servers")
    
    rospy.spin()

