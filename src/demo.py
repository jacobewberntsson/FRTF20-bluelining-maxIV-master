#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def callback(msg):
    
    pub.publish(msg)
    rospy.sleep(2)
    msg = Twist()
    pub.publish(msg)

rospy.init_node('demo', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
sub = rospy.Subscriber('/demo', Twist, callback)

rospy.spin()