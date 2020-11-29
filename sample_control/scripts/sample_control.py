#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetJointProperties
from gazebo_msgs.srv import ApplyJointEffort
from gazebo_msgs.srv import JointRequest
from std_msgs.msg import Float64
import math

Ts = 0.001
joint_name = 'joint1'
J = 0.58
omega = 2*math.pi*2

def get_ref_callback(data):
    global angle_ref
    angle_ref = data.data

if __name__=="__main__":
    rospy.init_node('sample_control')
    rate = rospy.Rate(1.0/Ts)

    rospy.Subscriber('angle_ref',Float64,get_ref_callback)

    get_angle = rospy.ServiceProxy('gazebo/get_joint_properties',GetJointProperties)
    clear_torque = rospy.ServiceProxy('gazebo/clear_joint_forces',JointRequest)
    set_torque = rospy.ServiceProxy('gazebo/apply_joint_effort',ApplyJointEffort)

    Kd = 3*omega**1*J
    Kp = 3*omega**2*J
    Ki = 1*omega**3*J

    torque = 0
    u_z = [0,0]
    y_z = [0,0]
    d_init = True

    angle_ref = 0

    while not rospy.is_shutdown():
        angle = get_angle(joint_name = joint_name).position[0]
        u_z[0] = angle_ref - angle
        y_z[0] = Ts*sum(u_z)/2 + y_z[1]


        if d_init:
            torque = Kp*u_z[0] + Ki*y_z[0]
            d_init = False
        else:
            torque = Kp*u_z[0] + Ki*y_z[0] + Kd*(u_z[0]-u_z[1])/Ts

        u_z[1] = u_z[0]
        y_z[1] = y_z[0]

        clear_torque(joint_name=joint_name)
        set_torque(joint_name=joint_name,effort=torque,duration=rospy.Duration(-1))

        rate.sleep()





