husky
=====

For Husky instructions and tutorials, please see [Robots/Husky](http://wiki.ros.org/Robots/Husky).

REAL ROBOT INSTRUCTIONS:

To use the Clearpath Robotics' Dual UR5 Husky, make sure you have your environment variables correctly set up. The correct setup (to be added in your real Husky's /etc/ros/setup.bash file (if you're using sim it will be your ~/.bashrc)) is listed below.

The correct way to bring up the arms and the moveit suite is to first bring up the arm drivers (ur_modern_driver ROS package), the planning execution node (which brings up move_group) and then RViz on the client computer which will interface to the robot. For the model to show up correctly you should have all drivers publishing on the robot, such as the flir_ptu, the arm drivers, the robotiq grippers, and the force torque. They should all be publishing their joint states from the drivers and then the model will show up neatly.

The commands to use a simple interface with the gripper:

1) Double check the gripper drivers are running

     rostopic list | grep gripper

2) Launch the simple gripper interface

     rosrun robotiq_s_model_control SModelSimpleController.py

3) For more complex control, see this  
     http://wiki.ros.org/robotiq/Tutorials/Control%20of%20an%20S-Model%20Gripper%20using%20the%20Modbus%20TCP%20Protocol

How to launch the planning execution node for the arms (while ssh'd inside robot)
  
  1) Launch the planning execution:
  
       roslaunch husky_dual_ur5_moveit_config husky_dual_ur5_planning_execution.launch real:=true

How to launch the arm drivers:

  1) Launch the drivers for left and right arms (after the arms have powered on and booted)
  
       roslaunch /etc/ros/indigo/husky-core.d/left_ur5.launch
      roslaunch /etc/ros/indigo/husky-core.d/right_ur5.launch

How to launch the RViz viewer on your computer:

REAL ROBOT INSTRUCTIONS:

To use the Clearpath Robotics' Dual UR5 Husky, make sure you have your environment variables correctly set up. The correct setup (to be added in your real Husky's /etc/ros/setup.bash file (if you're using sim it will be your ~/.bashrc)) is listed below.

The correct way to bring up the arms and the moveit suite is to first bring up the arm drivers (ur_modern_driver ROS package), the planning execution node (which brings up move_group) and then RViz on the client computer which will interface to the robot. For the model to show up correctly you should have all drivers publishing on the robot, such as the flir_ptu, the arm drivers, the robotiq grippers, and the force torque. They should all be publishing their joint states from the drivers and then the model will show up neatly.

The commands to use a simple interface with the gripper:

1) Double check the gripper drivers are running

     rostopic list | grep gripper

2) Launch the simple gripper interface

     rosrun robotiq_s_model_control SModelSimpleController.py

3) For more complex control, see this  
     http://wiki.ros.org/robotiq/Tutorials/Control%20of%20an%20S-Model%20Gripper%20using%20the%20Modbus%20TCP%20Protocol

How to launch the planning execution node for the arms (while ssh'd inside robot)
  
  1) Launch the planning execution:
  
       roslaunch husky_dual_ur5_moveit_config husky_dual_ur5_planning_execution.launch real:=true

How to launch the arm drivers:

  1) Launch the drivers for left and right arms (after the arms have powered on and booted)
  
       roslaunch /etc/ros/indigo/husky-core.d/left_ur5.launch
      roslaunch /etc/ros/indigo/husky-core.d/right_ur5.launch

How to launch the RViz viewer on your computer:

  1) Set up your ROS_MASTER_URI and ROS_IP:
  
      export ROS_MASTER_URI=http://192.168.1.11:11311
      export ROS_IP=your_computers_lan_ip
  
  2) Download the Dual UR5 Husky code and install on your computer:
  
      mkdir -p husky_ws/src
      cd husky_ws/src && catkin_init_workspace
      git clone https://github.com/husky/husky
      cd husky && git checkout dual_ur5_husky
      cd ~/husky_ws && catkin_make install
      source ~/husky_ws/devel/setup.bash
  
  3) Run the moveit_rviz.launch file:
  
     roslaunch husky_dual_ur5_moveit_config moveit_rviz.launch config:=true
  
 How to read data from the force torques:
 
 1) Check the sensors are running
 
     rostopic list | grep force_torque
 
 2) Echo the topic
 
     rostopic echo husky_left_gripper/robotiq_force_torque_sensor


VERSION 1.2 INSTRUCTIONS:

    Git clone all of the code from http://github.com/dualur5husky. Skip the pointgrey_camera_driver.
    Checkout husky's dual_ur5_husky branch
    Checkout robotiq's indigo-devel
    Install all of the dependencies and move the robotiq_s_model_articulated_gazebo* pkgs out of the workspace since we don't need gazebo on a real bot
    checkout dual_ur5_husky branch from the husky/ repo
    Build the workspace
    install moveit. apt-get install ros-indigo-moveit*
    apt-get install ros-indigo-ompl*
    apt-get install ros-indigo-moveit-ompl*
    apt-get install python-pymodbus ros-indigo-tf2-ros
    apt-get install ros-indigo-joint-state-controller ros-indigo-position-controllers
    install the robot
        rosrun robot_upstart install husky_bringup/launch/dual_ur5
        this will put everything in the /etc/ros/indigo/husky.d folder which will not get started on boot, but will help you with launching everything
    before launching the robot, checkout robotiq's jade-devel


bringing up the robot and controlling it:

    turn the robot on by pressing the husky power button on the back
    turn the arms on by pressing both buttons
    go to the arm tablet screen attached to the husky and make sure the robots are activated and un e-stopped
    head back to your computer and ssh into the robot via LAN or WLAN to 192.168.1.11, administrator/clearpath
    once logged in, go to the /etc/ros/indigo/husky.d folder.
    roslaunch ptu.launch twice. it will fail the first time for some unknown reason, then
    roslaunch *
    this will launch the 2 gripper launch files, the 2 arm launch files, and the ptu.launch which will enable all of the drivers for the robot
    Now, bringup the moveit planning execution
    roslaunch husky_dual_ur5_moveit_config husky_dual_ur5_moveit_planning_execution.launch real:=true
    now you can run whatever code you would like. for examples on how to control each part of the robot, see the code inside the "dual_ur5_teleop_general" package which implements some robot control in python.
    Alternatively, you can move the robot around using the dual_ur5_teleop_general ROS package and the joystick that ships with the husky. To do this:
    rosrun dual_ur5_teleop_general dual_ur5_teleop_general.py
    The controller scheme is listed below:


Dual UR5 Teleop General Controller Mappings:


Preface: This teleoperation uses MoveIt! to move the arms around. You may want to configure and tweak your MoveIt! settings inside the husky_dual_ur5_moveit_config package to better suit your research needs and your research environment. Things of note should be: setting the ground plane, setting padding for the obstacles/the arms, joint limits and joint speeds/accelerations, and what type of planning library.


Regular arm control mode:

    Press Left Trigger to enter "Left Arm" control mode. This will make all movements of the joysticks correspond to left arm movement.
    Press Right Trigger to enter "Right Arm" control mode. This will make all movements of the joystick correspond to the right arm movement.
    While in the respective arm control mode, each joystick direction will move the robot's end effector 5cm in the specified direction.
    Leftjoystick left: Move left 5cm
    Leftjoystick right: Move right 5cm
    Leftjoystick up: Move forward 5cm
    Leftjoystick down: Move backward 5cm
    Right joystick up: Move up 5cm
    Right joystick down: Move down 5cm
    Back: Will return the arm to its home position. Please have the e-stop ready as it may swing around rapidly depending on its starting position.


Tips and tricks: Note, the arm may "flail" and "whip" around if the specified direction is outside its current joint limit motion ranges. In doing so, unless you have the ground plane added in MoveIt, it MAY plan and execute through the ground. The arm will e-stop itself however, but always be careful, run on low speed, have RViz open to see the workspace, and have your hand on the e-stop at all times! You are responsible for any damage done to the arms or grippers, and not Clearpath Robotics.

Gripper control mode:

    Press "B" to enter Gripper control mode. This will make all joystick movements correspond to the grippers. They are:
    Right joystick left: Rotate left
    Right joystick right: Rotate right
    Press the Left Bumper to open and close the left gripper.
    Press the Right bumper to open and close the right gripper.
    
Individual Joint Control Mode:

While having the left or right arm selected using the left and right triggers respectively, you can press "Y" on the controller to move up the joint chain, and "X" to move down the joint chain. What this will do is allow you to control each joint of the arm individually using the right joystick. Moving the right joystick left or right will rotate that joint in its respective direction by a set step amount. These can be modified by editing the script. 

PTU Control Mode:

By using the DPad, you can control the direction and movement of the PTU. DPAD left  will move it left. Dpad UP will move it up. Dpad DOWN will move it down. And DPAD right will move it right a set step amount.

Home positions:

Depending on which arm you have selected, you can move it to its home position by pressing the "back" button on the controller. This will rest the arm on top of the robot and prepare it for movement or packing. Please stand clear as it stretches out about 4 feet and moves rapidly. It will e-stop immediately if it notices anything out of the normal.


usage of the ptu:

    launch the ptu.launch driver
    send joint state info to /ptu/cmd
        rostopic pub /ptu/cmd sensor_msgs/JointState (press tab to fill it in). The position array and the velocity array must have two values in each, one for pan, one for tilt. The range for angles is in radians
