import sys
import rospy
import numpy as np
import tf
import math
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from matplotlib.patches import Wedge
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8, Int16

obstacle_track_threshold = 0.10
obstacle_distance_threshold = 1.

plot = True
ax = None
mat = None
pos = None

lane_pub = None
speed_pub = None

def getClosestPoint(pos, laneID):
	if laneID == 1:
		p1 = (1.90, .80)
		p2 = (4.05, .79)
		p3 = (1.95, 3.58)
		p4 = (4.05, 3.58)
		c1 = (1.90, 2.22) 
		c2 = (4.15, 2.15)
		r1 = 1.27
		r2 = 1.31 
	if laneID == 2:
		p1 = (1.95, .48)
		p2 = (4.04, .48)
		p3 = (1.95, 3.88)
		p4 = (4.05, 3.88)
		c1 = (1.90, 2.22)
		c2 = (4.15, 2.15)
		r1 = 1.60
		r2 = 1.65
	if (pos[0] < p1[0]):
		angle = math.atan((pos[1] - c1[1])/(pos[0] - c1[0]))
		return (c1[0] + r1 * math.cos(angle), c1[1] + r1 * math.sin(angle))
	elif (pos[0] < c2[0]) and (pos[1] < c2[1]):
		return (pos[0], p1[1])
	elif (pos[0] < c2[0]) and (pos[1] >= c2[1]):
		return (pos[0], p3[1])
	else:
		angle = math.atan((pos[1] - c2[1])/(pos[0] - c2[0]))
		return (c2[0] + r2 * math.cos(angle), c2[1] + r2 * math.sin(angle))

def on_scan(scan_msg):
	if mat is None:
		return

	ranges = scan_msg.ranges
	out = np.zeros((360, 2))

	for inx, i in enumerate(ranges):
		if inx > 65 and not (inx > 295):
			continue
		inxr = inx*np.pi/180
		x = np.cos(inxr) * i
		y = np.sin(inxr) * i
		auto_vector = np.array([x, y, 0, 1]).reshape(4,1)
		world = np.dot(mat, auto_vector)
		out[inx, :] = world[:2, 0].reshape(1 ,2)

	out[np.isnan(out)] = 42
	out[np.isinf(out)] = 42

	closestPoints1 = np.array([getClosestPoint(o, 1) for o in out])
	closestPoints2 = np.array([getClosestPoint(o, 2) for o in out])
	distanceToTrack1 = np.array([np.linalg.norm(closestPoints1[i] - out[i]) for i in range(len(out))])
	distanceToTrack2 = np.array([np.linalg.norm(closestPoints2[i] - out[i]) for i in range(len(out))])
	closestPoints1 = closestPoints1[distanceToTrack1 < obstacle_track_threshold]
	closestPoints2 = closestPoints2[distanceToTrack2 < obstacle_track_threshold]
	closestPointCar = getClosestPoint([pos.x, pos.y], 1)
	distanceToCar1 = np.array([np.linalg.norm(closestPoints1[i] - closestPointCar) for i in range(len(closestPoints1))])
	distanceToCar2 = np.array([np.linalg.norm(closestPoints2[i] - closestPointCar) for i in range(len(closestPoints2))])
	if (len(distanceToCar1) != 0 and len(distanceToCar2) != 0 and np.min(distanceToCar1) < obstacle_distance_threshold and np.min(distanceToCar2) < obstacle_distance_threshold):
		speed_pub.publish(0)
	elif (len(distanceToCar1) != 0 and np.min(distanceToCar1) < obstacle_distance_threshold):
		lane_pub.publish(2)
	elif (len(distanceToCar2) != 0 and np.min(distanceToCar2) < obstacle_distance_threshold):
		lane_pub.publish(1)
	else:
		speed_pub.publish(200)

	if plot:
		prepareVisualization()
		ax.scatter(*out.T, color='g')
		ax.scatter(*closestPoints1.T, color='r')
		ax.scatter(*closestPoints2.T, color='r')
		plt.show(block=False)
		rospy.sleep(0.1)  # sleep to avoid threading issues with plotting

def prepareVisualization():
	ax.cla()
	angle = 90
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
	ax.plot(pos.x, pos.y, marker='o')
	#ax.plot([pos.x, pos.x + 0.2 * math.cos(yaw)], [pos.y, pos.y + 0.2 * math.sin(yaw)], color='g')

def on_odom(msg):
	global mat
	global pos
	pos = msg.pose.pose.position
	(r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
	mat = np.matrix([[np.cos(y), -np.sin(y), 0, pos.x], [np.sin(y), np.cos(y), 0, pos.y], [0, 0, 1, 0], [0, 0, 0, 1]])

def main():
	global ax
	global lane_pub
	global speed_pub
	rospy.init_node("obstacle_detect")
	fig, ax = plt.subplots()
	lane_pub = rospy.Publisher("/lane", UInt8, queue_size=1, latch=True)
	speed_pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=1, latch=True)
	rospy.Subscriber("/scan", LaserScan, on_scan, queue_size=1)
	pos_sub = rospy.Subscriber("/localization/odom/7", Odometry, on_odom, queue_size=1)
	if plot:
		plt.show()
	else:
		rospy.spin()

if __name__ == '__main__':
	main()