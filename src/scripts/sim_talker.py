#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('coordinates', String, queue_size=10)
    rospy.init_node('coordinates', anonymous=True)
    rate = rospy.Rate(10) # 1hz
    t = 0
    while not rospy.is_shutdown():
        t += 0.005
        postion = get_coords(t) #x,y,z
        rospy.loginfo(postion)
        pub.publish(postion)
        rate.sleep()
def get_coords(t):
    coords = ''
    coords += str((0.2 * math.cos(t))) + ','
    coords += str((1)) + ','
    coords += str((0.2 * math.sin(t))+ 0.5)
    return coords
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
