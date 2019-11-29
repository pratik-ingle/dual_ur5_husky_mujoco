import pybullet as p
import pybullet_data

#datapath = "/home/cong/ros"
p.connect(p.GUI)
#pybullet.setAdditionalSearchPath(datapath)
p.loadURDF("/home/cong/ros_ws/openai_ros_ws/src/bullet3/data/dual_ur5_husky_mujoco/husky_description/urdf/dual_arm_husky_pybullet.urdf", [0,0,1])
