#!/usr/bin/env python
# license removed for brevity
import rospy
from rotors_comm.msg import WindSpeed
import math
from geometry_msgs.msg import Vector3

def wind_generator():
    pub = rospy.Publisher('wind_speed', WindSpeed, queue_size=10)
    rospy.init_node('wind_generator', anonymous=True)
    
    Ax = rospy.get_param('Amplitude/x',0.1)
    Ay = rospy.get_param('Amplitude/y',0)
    Az = rospy.get_param('Amplitude/z',0)

    fx = rospy.get_param('Frequency/x',1)
    fy = rospy.get_param('Frequency/y',1)
    fz = rospy.get_param('Frequency/z',1)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = WindSpeed()
        msg.velocity = Vector3(x=Ax * math.sin(2 * math.pi * fx* rospy.get_time()),
                              y=Ay * math.sin(2 * math.pi * fy* rospy.get_time()),
                              z=Az * math.sin(2 * math.pi * fz* rospy.get_time()))
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        wind_generator()
    except rospy.ROSInterruptException:
        pass