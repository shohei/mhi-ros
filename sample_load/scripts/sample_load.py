#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import math

Ts = 1

if __name__=="__main__":
    rospy.init_node('sample_load')
    rate = rospy.Rate(1.0/Ts)

    pub = rospy.Publisher('angle_ref',Float64,queue_size=10)
    ref = 0

    while not rospy.is_shutdown():
        pub.publish(ref)
        ref += (Ts*2*math.pi/60.0)
        rate.sleep()

