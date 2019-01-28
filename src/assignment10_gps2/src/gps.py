import rospy
import sys
import signal
import math
import matplotlib.pyplot as plt
import tf
from matplotlib.patches import Wedge
from sympy import *
import numpy as np
from sympy.geometry import *
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8

steering_pub = None
plot = False

# terrible code ahead
def getClosestPoint(pos, distance, laneID):
	if laneID == 1:
		p1 = (1.90, .82)
		p2 = (4.05, .82)
		p3 = (1.95, 3.56)
		p4 = (4.05, 3.52)
		c1 = (2.00, 2.20) 
		c2 = (4.05, 2.15)
		r1 = 1.27
		r2 = 1.31 
		# Verwendete Koordinaten entsprechen nicht ganz dem was wir in dem Bild gemessen haben fuehren so aber zu deutlich besseren Ergebnissen
	if laneID == 2:
		p1 = (1.95, .48)
		p2 = (4.04, .48)
		p3 = (1.95, 3.84)
		p4 = (4.05, 3.84)
		c1 = (2.00, 2.20)
		c2 = (4.05, 2.15)
		r1 = 1.60
		r2 = 1.65
	if (pos.x < p1[0]):
		angle = math.atan((pos.y - c1[1])/(pos.x - c1[0]))
		diff = distance / (2 * math.pi * r1)
		angle_diff = diff * 2 * math.pi
		angle_res = angle + angle_diff + math.pi
		if angle_res < (3 * math.pi / 2):
			return (c1[0] + r1 * math.cos(angle_res), c1[1] + r1 * math.sin(angle_res))
		else:
			new_diff = angle_diff - (angle_res - (3 * math.pi / 2))
			diff = new_diff / angle_diff
			distance = distance - (diff * distance)
			return (p1[0] + distance, p1[1])
	elif (pos.x < c2[0]) and (pos.y < c2[1]):
		if pos.x + distance < c2[0]:
			return (pos.x + distance, p1[1])
		else:
			distance = pos.x + distance - c2[0]
			angle = 3 * math.pi / 2
			diff = distance / (2 * math.pi * r2)
			angle_diff = diff * 2 * math.pi
			angle_res = angle + angle_diff
			return (c2[0] + r2 * math.cos(angle_res), c2[1] + r2 * math.sin(angle_res))
	elif (pos.x < c2[0]) and (pos.y >= c2[1]):
		if pos.x - distance >= p3[0]:
			return (pos.x - distance, p3[1])
		else:
			distance = c1[0] - (pos.x - distance)
			angle = math.pi / 2
			diff = distance / (2 * math.pi * r1)
			angle_diff = diff * 2 * math.pi
			angle_res = angle + angle_diff
			return (c1[0] + r1 * math.cos(angle_res), c1[1] + r1 * math.sin(angle_res))
	elif (pos.x <= 6.00):
		angle = math.atan((pos.y - c2[1])/(pos.x - c2[0]))
		diff = distance / (2 * math.pi * r2)
		angle_diff = diff * 2 * math.pi
		angle_res = angle + angle_diff
		if angle_res < (math.pi / 2):
			return (c2[0] + r2 * math.cos(angle_res), c2[1] + r2 * math.sin(angle_res))
		else:
			new_diff = angle_diff - (angle_res - (math.pi / 2))
			diff = new_diff / angle_diff
			distance = distance - (diff * distance)
			return (p4[0] - distance, p4[1])

def closestPointLine(pos, line):
	perp = line.perpendicular_line(pos)
	intersect = line.intersection(perp)[0]
	dist = intersect.distance(pos)
	return (intersect, float(dist))

def on_odom(msg):
	global ax
	pos = msg.pose.pose.position
	(r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
	if plot:
		prepareVisualization()
	res = getClosestPoint(pos, 0.5, 1)
	angle = math.atan2((res[1] - pos.y), (res[0] - pos.x))
	#print("Yaw: ", y)
	#print("Target: ", angle)
	angle_diff = math.atan2(math.sin(angle - y), math.cos(angle - y)) * (-1)
	map_steering(angle_diff)
	if plot:
		ax.plot(pos.x, pos.y, marker='o')
		ax.plot([pos.x, pos.x + 0.2 * math.cos(y)], [pos.y, pos.y + 0.2 * math.sin(y)], color='g')
		ax.plot(res[0], res[1], marker='o', color='r')
		plt.show(block=False)
		rospy.sleep(0.01)

def prepareVisualization():
	global ax
	angle = 90
	if ax is None:
		ax = plt.gca()
	ax.cla()
	theta1, theta2 = angle, angle + 180
	w1 = Wedge((1.95, 2.15), 1.20, theta1, theta2, fill=False)
	w2 = Wedge((4.05, 2.15), 1.20, theta2, theta1, fill=False)
	w3 = Wedge((1.95, 2.15), 1.84, theta1, theta2, fill=False)
	w4 = Wedge((4.05, 2.15), 1.84, theta2, theta1, fill=False)
	w5 = Wedge((1.95, 2.15), 1.36, theta1, theta2, fill=False, color='b')
	w6 = Wedge((4.05, 2.15), 1.36, theta2, theta1, fill=False, color='b')
	w7 = Wedge((1.95, 2.15), 1.68, theta1, theta2, fill=False, color='b')
	w8 = Wedge((4.05, 2.15), 1.68, theta2, theta1, fill=False, color='b')
	for wedge in [w1, w2, w3, w4, w5, w6, w7, w8]:
		ax.add_artist(wedge)
	ax.set_xlim([0.,6.00])
	ax.set_ylim([0.,4.30])
	ax.plot([1.95,4.05], [.95, .95], color='k')
	ax.plot([1.95,4.05], [.31, .31], color='k')
	ax.plot([1.95,4.05], [3.36, 3.36], color='k')
	ax.plot([1.95,4.05], [4.00, 4.00], color='k')
	ax.plot([1.95,4.05], [.46, .46], color='b')
	ax.plot([1.95,4.05], [3.84, 3.84], color='b')
	ax.plot([1.95,4.05], [.78, .78], color='b')
	ax.plot([1.95,4.05], [3.52, 3.52], color='b')

def main(args):
	global fix, ax
	global steering_pub
	rospy.init_node("navigation", anonymous=True);
	fig, ax = plt.subplots()
	steering_pub = rospy.Publisher("/steering", UInt8, queue_size=100, latch=True)
	pos_sub = rospy.Subscriber("/localization/odom/4", Odometry, on_odom, queue_size=1)
	if plot:
		plt.show()
	else:
		rospy.spin()

def map_steering(angle_diff):
	steering = 2 * angle_diff * 180 / math.pi + 90
	#print("Steering", steering)
	steering = np.clip(steering, 0, 180)
	steering_pub.publish(steering)

def tempmain(args):
	print(getClosestPoint(Point2D(1, 1), 0.2, 2))

if __name__ == '__main__':
	main(sys.argv)