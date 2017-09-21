#!/usr/bin/env python
import rospy
import random

from provider_hydrophone.msg import PingMsg


def sim_power():
    rospy.init_node('sim_hydro', anonymous=True)
    pub = rospy.Publisher('/provider_hydrophone/ping', PingMsg, queue_size=10)
    rate = rospy.Rate(1)  # 10hz
    msg = PingMsg()
    while not rospy.is_shutdown():

        msg.amplitude = random.uniform(0, 100)
        msg.frequency = random.uniform(20, 40)
        msg.heading = random.uniform(0, 360)
        msg.elevation = random.uniform(0, 90)
        msg.noise = random.uniform(30, 40)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        sim_power()
    except rospy.ROSInterruptException:
        pass