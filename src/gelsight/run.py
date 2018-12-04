#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

def talker():
    pub = rospy.Publisher('gelsight', String, queue_size=1)
    rospy.init_node('gelsight_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz



    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass