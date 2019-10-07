# Dual-UR5-Husky-MuJoCo model

We will create a dual ur5 husky mujoco model based on original dual-ur5-husky reps. For the Gazebo demo and some related setting, you can find [here](http://www.clearpathrobotics.com/assets/guides/husky/HuskyDualManip.html).

What we need for the mojuco model is just the stl file and urdf file. So first, download all the needed packages as follows:
```bash
mkdir -p ~/dual_ws/src
cd ~/dual_ws/src && catkin_init_workspace
git clone https://github.com/DualUR5Husky/husky
git clone https://github.com/DualUR5Husky/ur_modern_driver
git clone https://github.com/DualUR5Husky/universal_robot
git clone https://github.com/DualUR5Husky/robotiq
git clone https://github.com/DualUR5Husky/husky_simulator
git clone https://github.com/DualUr5Husky/flir_ptu
```
Some official reps may update, like [universal_robot](https://github.com/ros-industrial/universal_robot), [robotiq](https://github.com/ros-industrial/robotiq). If you want to control them for the real robot, please update the latest drivers. But for now, we just use the husky reps for modelling.

For the Gazebo simulation: ![dual-ur5-husky-in-Gazebo](http://www.clearpathrobotics.com/assets/guides/husky/_images/Selection_192.png).

There are some points you need to notice for mujoco modelling.

1. prepare the ingredient for urdf file.

From [here](), we can know that urdf is different from mujoco mjcf file. So first, we should do some pre-processing.

- mesh file

For the URDF file, there usually have two mesh files: dae file for visualization and stl file for collision. However, from [here](http://www.mujoco.org/forum/index.php?threads/unknown-mesh-file-type-dae.3495/), we can know that mujoco cannot import .dae file, so we should convert the .dae file to .stl file. You can use [MeshLab](http://www.meshlab.net/) to convert and it is very easy to use. But if you have a lot of mesh file to convert, ...

Then, we should put all the mesh files to a folder that mujoco can find them. P.S. some related questions here: [Multiple mesh folders](http://www.mujoco.org/forum/index.php?threads/multiple-mesh-folders.3720/) 

Here, I create a folder named [meshes-mujoco]() and put all the related mesh files in it (every mesh file has uniq name).

- change the urdf/xacro file

change every .dae file to .stl file, using the new mesh file converted from MeshLab.

- add some mujoco tags into the urdf file

From the top robot urdf tags, we need to add some mujoco tags. For my robot, the **meshdir**, **balanceinertia**, **discardvisual** tags are needed. For the details of the tags, [click here](http://www.mujoco.org/book/XMLreference.html#compiler).

Add **balanceinertia** tag if you get the error:
```bash
Error: inertia must satisfy A + B >= C; use 'balanceinertia' to fix
Object name = inertial_link, id = 3
```

If you want your robot look good, not just simple geometry, add **discardvisual** tag. The tag default is true, so it discard the visual stl file and change some complex mesh file to the simple convexhull geometry, like box, cylinder. Because for the pysical simulator, it just use the simple geometry to calculate or collision detection. ROS also has the charactor, using FCL for collision detection.

From [here (some meshes ignored when converting urdf to mjcf)](http://www.mujoco.org/forum/index.php?threads/meshes-ignored-when-converting-urdf-to-mjcf.3433/):

`
Darwin has collision meshes, which is why they are showing. *Sawyer only has visual meshes, which are discarded automatically when discardvisual="true", which is the default when parsing URDFs. So you need to set **discardvisual="false"** in \<compiler> tag*. Also, you have a box with size="0 0 0" which is an error. Geom sizes must be positive. See attached model.

Note that collision geoms are placed in geom group 0, while visual geoms are placed in geom group 1. You can toggle the rendering of each group **pressing '0' and '1'** respectively (in the GUI, press "0" and "1", you can see the collision rendering(simple geometry) or visual rendering(beautiful)). 
`

```xml
  <mujoco>
        <compiler 
        meshdir="../meshes_mujoco/" 
        balanceinertia="true" 
        discardvisual="false" />
  </mujoco>
```



- check urdf file


- convert urdf to mjcf file


- change the raw xml file to adapt the mujoco

[Here](https://github.com/openai/mujoco-py/issues/216) introduces some related thing to change.











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
