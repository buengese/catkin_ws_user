import rospy
import sys
import signal
import math
import matplotlib.pyplot as plt
from sympy import *
import numpy as np
from sympy.geometry import *
from nav_msgs.msg import Odometry

dist = 0.
distsq = 0.
amount = 0
poses = []

def closestPointCircle(pos, center, radius, temp):
	line = Line(center, pos)
	circle = Circle(center, radius)
	intersect = circle.intersection(line)[temp]
	dist = intersect.distance(pos)
	if center.distance(pos) < radius:
		return (intersect, float(dist) * (-1))
	return (intersect, float(dist))

def closestPointLine(pos, line):
	perp = line.perpendicular_line(pos)
	intersect = line.intersection(perp)[0]
	dist = intersect.distance(pos)
	return (intersect, float(dist))

def on_odom(msg):
	global amount
	global dist
	global distsq
	global poses
	amount = amount + 1
	pos = msg.pose.pose.position
	poses.append((pos.x, pos.y))
	if (pos.x < 1.95) and (pos.y < 2.15):
		res = closestPointCircle(Point(pos.x, pos.y), Point(1.95, 2.15), 1.21, 0)
		dist = dist + res[1]
		distsq = distsq + math.pow(dist, 2)
	elif (pos.x < 1.95) and (pos.y >= 2.15):
		res = closestPointCircle(Point(pos.x, pos.y), Point(1.95, 2.15), 1.21, 0)
		dist = dist + res[1]
		distsq = distsq + math.pow(dist, 2)
	elif (pos.x < 4.05) and (pos.y < 2.15):
		res = closestPointLine(Point(pos.x, pos.y), Line(Point(1.95, 0.95), Point(4.04, 0.95)))
		distsq = distsq + math.pow(dist, 2)
		if pos.y > 0.95:
			dist = dist + (res[1]* (-1))
		else:
			dist = dist + res[1]
	elif (pos.x < 4.05) and (pos.y >= 2.15):
		res = closestPointLine(Point(pos.x, pos.y), Line(Point(1.95, 3.36), Point(4.04, 3.36)))
		distsq = distsq + math.pow(dist, 2)
		if pos.y < 3.36:
			dist = dist + (res[1] * (-1))
		else:
			dist = dist + res[1]
	elif (pos.x <= 6.00) and (pos.y < 2.15):
		res = closestPointCircle(Point(pos.x, pos.y), Point(4.05, 2.15), 1.21, 1)
		distsq = distsq + math.pow(dist, 2)
		dist = dist + res[1]
	elif (pos.x <= 6.00) and (pos.y >= 2.15):
		res = closestPointCircle(Point(pos.x, pos.y), Point(4.05, 2.15), 1.21, 1)
		distsq = distsq + math.pow(dist, 2)
		dist = dist + res[1]
	else:
		print("unhandled position")

def signal_handler(sig, frame):
	global dist
	global distsq
	global amount
	global poses
	points = np.asarray(poses)
	plt.scatter(*points.T)
	plt.show()

	print("Avg dist: ", dist / amount)
	print("Avg squared dist: ", distsq / math.pow(amount, 2))
	rospy.signal_shutdown("End")

def main(args):
	rospy.init_node("navigation", anonymous=True);
	pos_sub = rospy.Subscriber("/localization/odom/4", Odometry, on_odom, queue_size=1)
	signal.signal(signal.SIGINT, signal_handler)
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)