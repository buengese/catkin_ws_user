import sys
import rospy
import numpy as np
import tf
import math
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from matplotlib.patches import Wedge
from sensor_msgs.msg import LaserScan

plot = True
ax = None
mat = None
pos = None
yaw = None

wall1_angle = np.pi

def on_scan(scan_msg):
	'''radius = np.asarray(scan_msg.ranges)
	angles = np.arange(scan_msg.angle_min, scan_msg.angle_max + scan_msg.angle_increment / 2, scan_msg.angle_increment)

	mask_fin = np.isfinite(radius)  # only consider finite radii
	#angle_spread_method1 = np.pi / 8  # size of cone to consider

	#mask_angle1 = np.logical_or(angles < (angle_spread_method1 - wall1_angle), angles > (wall1_angle - angle_spread_method1))
	#mask_front = np.logical_and(mask_fin, mask_angle1)

	masked_angles_front = angles[mask_fin]
	masked_radius_front = radius[mask_fin]

	x = np.cos(masked_angles_front) * masked_radius_front
	y = np.sin(masked_angles_front) * masked_radius_front
	ones = np.ones(len(x))
	zeros = np.zeros(len(x))
	points_front = np.column_stack((x, y, zeros, ones))
	for p, i in enumerate(points_front):
		world = np.dot(mat, p)
		print(world)'''
	#points_out = points_out[:,:2]
	#print(points_out)
	ranges = scan_msg.ranges
	out = np.zeros((360, 2))

	for inx, i in enumerate(ranges):
		print(inx)
		inxr = inx*np.pi/180
		x = np.cos(inxr) * i
		y = np.sin(inxr) * i
		auto_vector = np.array([x, y, 0, 1]).reshape(4,1)
		world = np.dot(mat, auto_vector)
		out[inx, :] = world[:2, 0].reshape(1 ,2)

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
	ax.scatter(*out.T, color='g')

	plt.show(block=False)
	rospy.sleep(0.1)  # sleep to avoid threading issues with plotting

def on_odom(msg):
	global mat
	global pos
	global yaw
	pos = msg.pose.pose.position
	(r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
	mat = np.matrix([[np.cos(y), -np.sin(y), 0, pos.x], [np.sin(y), np.cos(y), 0, pos.y], [0, 0, 1, 0], [0, 0, 0, 1]])

def main():
	global ax
	rospy.init_node("collision")
	fig, ax = plt.subplots()
	rospy.Subscriber("/scan", LaserScan, on_scan, queue_size=1)
	pos_sub = rospy.Subscriber("/localization/odom/7", Odometry, on_odom, queue_size=1)
	if plot:
		plt.show()
	else:
		rospy.spin()

if __name__ == '__main__':
	main()