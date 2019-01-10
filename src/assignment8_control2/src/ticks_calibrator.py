import rospy
from std_msgs.msg import Int32, Int16, UInt8, Float32

ticks = 0

def on_tick(msg):
	global ticks
	if ticks == 0:
		speed_pub.publish(300)
	if ticks == 100:
		print "done"
		speed_pub.publish(0)
	ticks += msg.data

rospy.init_node("ticks_calibration", anonymous=True)

ticks_sub = rospy.Subscriber("/ticks", UInt8, on_tick, queue_size=1)
speed_pub = rospy.Publisher('/speed', Int16, queue_size=1)

rospy.spin()
