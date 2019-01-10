import rospy
from std_msgs.msg import Int32, Int16, UInt8, Float32
import math
import time
import sys

class Speedometer(object):

	def __init__(self, arg):
		self.period = arg
		self.ticks = []
		self.ticks_sub = rospy.Subscriber("/ticks", UInt8, self.on_tick, queue_size=1)

	def get_tps(self):
		if len(self.ticks) == 0:
			return 0

		return float(sum(t[1] for t in self.ticks))/len(self.ticks)

	def get_mps(self):
		return self.get_tps() * 0.0115 # distance per tick

	def on_tick(self, msg):
		now = rospy.get_time()
		read = msg.data
		self.ticks.append((now, read))

		too_old = lambda t: now - t[0] > self.period

		self.ticks = [tick for tick in self.ticks if not too_old(tick)]

def main(args):
	rospy.init_node('Speedometer', anonymous=True)
	speed = Speedometer(1.)

	pub_mps = rospy.Publisher('/mps', Float32, queue_size=1)
	
	while not rospy.is_shutdown():
		print speed.get_tps()
		pub_mps.publish(speed.get_mps())

	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)