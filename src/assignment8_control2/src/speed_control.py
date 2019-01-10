import rospy
from std_msgs.msg import Int32, Int16, UInt8, Float32
import sys
import numpy as np

class SpeedControl(object):

	def __init__(self):
		self.k_d = 0.1
		self.k_p = 0.1
		self.target_speed_diff = 0.

		self.current_speed = 0
		self.target_speed = 0
		self.current_diff = 0

		self.speed_list = []

		self.mps_sub = rospy.Subscriber('/mps', Float32, self.on_mps, queue_size=1)
		self.target_sub = rospy.Subscriber('/speed_control/target', Float32, self.on_target, queue_size=1)
		self.speed_pub = rospy.Publisher('/speed', Int16, queue_size=1)

	def on_mps(self, msg):
		self.current_speed = msg.data

		self.speed_list.append(msg.data)
		if len(self.speed_list) > 3:
			self.speed_list = self.speed_list[:-3]

		if len(self.speed_list) > 2:
			self.calc_diff()

	def calc_diff(self):
		arr = np.array(self.speed_list)
		grad = np.gradient(arr)
		self.current_diff = np.mean(grad)

	def on_target(self, msg):
		self.target_speed = msg.data

	def map_speed(self, target):
		tps = (target/0.0115)
		return tps * 289 # tps to motor rpm, not really accurate in my testing correlation between ticks per second and rpm wasn't very linear

	def control(self):
		u_t = self.target_speed
		u_p = self.k_p * (self.target_speed - self.current_speed) 
		u_d = self.k_d * (self.target_speed_diff - self.current_diff)
		result_speed = self.map_speed(u_t)
		print result_speed
		self.speed_pub.publish(result_speed)
		
def main(
	args):
	rospy.init_node("speed_control", anonymous=True)
	sc = SpeedControl()
	while not rospy.is_shutdown():
		sc.control()

if __name__ == '__main__':
	main(sys.argv)