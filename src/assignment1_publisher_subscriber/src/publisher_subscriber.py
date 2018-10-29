#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32

def callback(data):
   message = "I heard %s" % data.data
   pub.publish(message)
   # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
   # In ROS, nodes are uniquely named. If two nodes with the same
   # node are launched, the previous one is kicked off. The
   # anonymous=True flag means that rospy will choose a unique
   # name for our 'listener' node so that multiple listeners can
   # run simultaneously.
   rospy.init_node('yaw_subscriber', anonymous=True)
   rospy.Subscriber("yaw", Float32, callback)
   # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()

if __name__ == '__main__':
    global pub
    pub = rospy.Publisher('assignment1_publisher_subscriber', String, queue_size=10)
    listener()
