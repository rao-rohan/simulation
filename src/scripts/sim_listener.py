#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os
import sys
import errno
import pybullet as p
import pybullet_data
import time
from time import sleep
import math
from math import pi
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf")
p.setRealTimeSimulation(False)
p.setGravity(0, 0, -10)
# load arm
model_path = "jaco_robotiq_object.urdf"
arm_id = p.loadURDF(model_path, [0, 0, 0], useFixedBase=True)
# variables specific to this arm
eef_index = 8
num_joints = 18
rp = [-pi / 2, pi * 5 / 4, 0., pi / 2, 0., pi * 5 / 4, -pi / 2, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
ll = [-pi] * num_joints
ul = [pi] * num_joints
jd = [0.000001] * num_joints
jr = np.array(ll) - np.array(ul)
ik_solver = p.IK_DLS
 # configure arm
for i in range(num_joints):
    p.resetJointState(arm_id, i, rp[i])
    p.enableJointForceTorqueSensor(arm_id, i)

# extract joint names
joint_id = {}
for i in range(p.getNumJoints(arm_id)):
    jointInfo = p.getJointInfo(arm_id, i)
    joint_id[jointInfo[1].decode('UTF-8')] = jointInfo[0]

# adjust block mass
weight_joint_id = joint_id["weight_joint"]
p.changeDynamics(arm_id, weight_joint_id, mass=0.5)

# set up variables needed during sim
len_seconds = 5
# number of steps to wait before recording torques
burn_in = 150
start_time = time.time()
controlled_joints = list(range(eef_index-1))
num_controlled_joints = eef_index-1
orn = p.getQuaternionFromEuler([math.pi / 2, math.pi / 2, math.pi / 2])
targetVelocities = [0] * num_controlled_joints
forces = [500] * num_controlled_joints
positionGains = [0.03] * num_controlled_joints
velocityGains = [1] * num_controlled_joints


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    p.stepSimulation()
    time.sleep(0.025)

    # move the arm
    pos = data.data.split(',')
    #print(pos)
    type(pos[0])
    arrPos = [float(pos[0]),float(pos[1]),float(pos[2])]
    #print(data.data)
    jointPoses = p.calculateInverseKinematics(arm_id, eef_index, arrPos, orn,
                                                lowerLimits=ll,
                                                upperLimits=ul,
                                                jointRanges=jr,
                                                restPoses=rp,
                                                jointDamping=jd,
                                                solver=ik_solver)[:num_controlled_joints]
    p.setJointMotorControlArray(bodyIndex=arm_id,
                                jointIndices=controlled_joints,
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=jointPoses,
                                targetVelocities=targetVelocities,
                                forces=forces,
                                positionGains=positionGains,
                                velocityGains=velocityGains)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('coordinates', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
