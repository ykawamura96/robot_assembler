#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from math import pi
import subprocess
import numpy
from std_srvs.srv import Empty
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import Imu
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class PID(object):
    
    def __init__(self):
        rospy.init_node('test', anonymous=True)
        self.kp = 0.08
        self.kd = 0.0001
        self.ki = 0.0
        self.last_pitch = 0
        self.command_topic = '/fullbody_controller/command'
        self.subscribe_topic = '/imu_sensor'
        self.pub = rospy.Publisher('/fullbody_controller/command', JointTrajectory, queue_size=10)
        self.t = 0
        (self.r_wheel, self.l_wheel) = (0, 0)
        rospy.Subscriber(self.subscribe_topic, Imu, self.callback)

    def callback(self, imu_data):
        # from IPython import embed; embed()
        q = imu_data.orientation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pitch = pitch
        print ("angle: {}".format(pitch))

        self.traj_msg = JointTrajectory()
        self.traj_msg.joint_names =  ['JOINT0', 'JOINT1', 'JOINT2', 'JOINT3', 'JOINT4', 'JOINT5', 'JOINT6', 'JOINT7', 'JOINT8', 'JOINT9', 'JOINT10', 'JOINT11', 'JOINT12', 'JOINT13']
        self.r_wheel, self.l_wheel = self.calc_target()
        self.traj_msg.points.append(JointTrajectoryPoint(positions=[0, 0, 0, 0, 0, 0, self.r_wheel, 0, 0, 0, 0, 0, 0, self.l_wheel],
                                                               time_from_start = rospy.Duration(0.01)))
        self.traj_msg.header.stamp = rospy.Time.now() # + rospy.Duration(0.1)
        self.pub.publish(self.traj_msg)
        # self.r_wheel -= 0.01
        # self.l_wheel += 0.01
        # print(self.r_wheel, self.l_wheel)

    def calc_target(self):
        target = 0
        up_thre = 1
        under_thre = -1
        cur_pitch = self.pitch
        
        diff = target - cur_pitch
        ddiff = self.last_pitch - cur_pitch
        idff = self.idiff
        self.idiff += diff
        self.last_pitch = cur_pitch
        u  =  self.kp * diff + self.kd * ddiff + self.ki * self.idiff
        return self.r_wheel + u, self.l_wheel - u
        
    def run(self):
        rospy.spin()

def main():
    rospy.wait_for_service('/gazebo/reset_world')
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    rep = reset_world()
    pid = PID()
    pid.run()

if __name__ == "__main__":
    main()
