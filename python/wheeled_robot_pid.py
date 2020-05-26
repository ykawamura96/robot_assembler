#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from math import pi
import subprocess
import numpy
from gazebo_msgs.msg import LinkStates
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def rad2deg(rad):
    return rad / pi / 2 * 360


class PIDController(object):

    def __init__(self):
        rospy.init_node('pid', anonymous=True)
        # subprocess.call(['rosservice','call','/gazebo/reset_world'], shell=True)
        self.kp = 0.3
        self.kd = 0.001
        self.ki = 0.0
        self.idiff = 0
        self.pitch = 0
        self.last_pitch = 0
        self.r_wheel = 0
        self.l_wheel = 0
        rospy.Subscriber('gazebo/link_states', LinkStates, self.callback)
        self.act_client = actionlib.SimpleActionClient('/fullbody_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.act_client.wait_for_server()
        self.traj_msg = FollowJointTrajectoryGoal()
        self.traj_msg.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        self.traj_msg.trajectory.joint_names = ['JOINT0', 'JOINT1', 'JOINT2', 'JOINT3', 'JOINT4', 'JOINT5', 'JOINT6', 'JOINT7', 'JOINT8', 'JOINT9', 'JOINT10', 'JOINT11', 'JOINT12', 'JOINT13']
        rospy.loginfo("wheel target : {}".format(self.r_wheel))
        self.traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0, 0, -0.6, 1.52, -0.6, 0, self.r_wheel, 0, 0, -0.6, 1.52, -0.6, 0, self.l_wheel],
                                                               time_from_start = rospy.Duration(0.1)))
        # send to robot arm
        self.act_client.send_goal(self.traj_msg)
        self.act_client.wait_for_result()

    def run(self):
        rospy.spin()

    def callback(self, data):
        link0 = data.pose[1]
        q = [link0.orientation.x, link0.orientation.y,link0.orientation.z, link0.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(q)
        self.pitch = pitch
        print ("angle: {}".format(pitch))
        self.act_client = actionlib.SimpleActionClient('/fullbody_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.act_client.wait_for_server()
        self.traj_msg = FollowJointTrajectoryGoal()
        self.traj_msg.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        self.traj_msg.trajectory.joint_names = ['JOINT0', 'JOINT1', 'JOINT2', 'JOINT3', 'JOINT4', 'JOINT5', 'JOINT6', 'JOINT7', 'JOINT8', 'JOINT9', 'JOINT10', 'JOINT11', 'JOINT12', 'JOINT13']
        self.r_wheel, self.l_wheel = self._calc_target()
        rospy.loginfo("wheel target : {}".format(self.r_wheel))
        self.traj_msg.trajectory.points.append(JointTrajectoryPoint(positions=[0, 0, -0.6, 1.52, -0.6, 0, self.r_wheel, 0, 0, -0.6, 1.52, -0.6, 0, self.l_wheel],
                                                               time_from_start = rospy.Duration(0.01)))
        self.act_client.send_goal(self.traj_msg)
        self.act_client.wait_for_result()

    def _calc_target(self):
        target_angle = 0
        diff = target_angle - self.pitch
        ddiff = self.last_pitch - self.pitch
        idff = self.idiff
        target_diff  =  self.kp * diff + self.kd * ddiff + self.ki * self.idiff
        self.idiff += diff
        self.last_pitch = self.pitch
        print("target_diff = {}".format(target_diff))
        return self.r_wheel + target_diff, self.l_wheel - target_diff
        
def main():
    PC = PIDController()
    PC.run()

if __name__ == '__main__':
    main()
    
