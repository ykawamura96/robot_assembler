#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from math import pi
import subprocess
import numpy
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelStates

POSITION_CONTROL_MODE = True
def rad2deg(rad):
    return rad / pi / 2 * 360

class PIDController(object):

    def __init__(self):
        rospy.init_node('pid', anonymous=True)
        self.KP = 0.80
        self.KD = 5.20
        self.KI = 0.0
        self.YAW_KP = 0.05
        self.POS_KP = 3.90
        self.POS_KD = 0.00
        self.idiff = 0.001
        self.pitch = 0
        self.pos_x = 0
        self.last_pitch = 0
        self.r_wheel = 0
        self.l_wheel = 0
        self.command_topics = ['/wheeled_robot/joint{}_position_controller/command'.format(i) for i in range (14)]
        self.command_pub = [rospy.Publisher(self.command_topics[i], Float64, queue_size=10) for i in range(14)]
        self.t = 0        
        rospy.Subscriber('/imu_sensor', Imu, self.callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.position_callback)

    def run(self):
        rospy.spin()

    def position_callback(self, data):
        idx = data.name.index('AssembledRobot')
        robot_pose = data.pose[idx]
        self.last_pos = self.pos_x
        self.pos_x = robot_pose.position.x
        ## need some fileter for velocity?
        self.vel_x = self.pos_x - self.last_pos
        
    def callback(self, data):
        q = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(q)
        self.pitch = pitch
        self.yaw = yaw
        rospy.loginfo("current angle: {}".format(pitch))
        self.r_wheel, self.l_wheel = self._calc_target()
        self._command_angle_to_wheels(self.r_wheel, self.l_wheel)
        rospy.loginfo("wheel target : {}".format(self.r_wheel))

    def _calc_target(self):
        target_angle = 0.00
        target_pos = 0.02
        diff = target_angle - self.pitch
        target_yaw = 0.0
        target_vel = 0.0
        yaw_diff = target_yaw - self.yaw
        vel_diff = target_vel - self.vel_x
        ddiff = self.last_pitch - self.pitch
        idff = self.idiff
        posdiff = target_pos - self.pos_x

        r_target_diff  =  self.KP * diff + self.KD * ddiff + self.KI * self.idiff - self.YAW_KP * yaw_diff + self.POS_KP * posdiff + self.POS_KD * vel_diff
        l_target_diff  =  self.KP * diff + self.KD * ddiff + self.KI * self.idiff + self.YAW_KP * yaw_diff + self.POS_KP * posdiff + self.POS_KD * vel_diff
        self.idiff += diff
        self.last_pitch = self.pitch
        if diff > 1.4 or diff < -1.4:
            return self.r_wheel, self.l_wheel
        else:
            return self.r_wheel + r_target_diff , self.l_wheel - l_target_diff

    def _command_angle_to_wheels(self, r, l):
        self.command_pub[6].publish(r)
        self.command_pub[13].publish(l)
        rospy.loginfo("right: {} \t left: {}\n".format(r, l))
        
def main():
    subprocess.call(["rosservice", "call", "/gazebo/reset_world"])
    PC = PIDController()
    PC.run()

if __name__ == '__main__':
    main()

    