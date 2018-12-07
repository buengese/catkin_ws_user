#!/usr/bin/env python2
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, UInt8, UInt16
from collections import namedtuple
from sympy import Symbol, nonlinsolve

from time import localtime, strftime

mask_angles = True  # whether to mask any angle that's not on the right side of the car

inlier_dist = 0.02  # max distance for inliers (in meters) since walls are very flat this can be low
sample_count = 50  # number RANSAC samples to take
steering_angle = 0 

wall1_angle = np.pi / 2
wall2_angle = 0

plotting = True  # whether to plot output

offline = True
start = False

pub_steering = rospy.Publisher('/steering', UInt8, queue_size=100, latch=True)
pub_speed = rospy.Publisher('/speed', Int16, queue_size=100, latch=True)

LineInfo = namedtuple('LineInfo', ['slope1', 'intercept1', 'wall_dist1', 'slope2', 'intercept2', 'wall_dist2', 'stamp'])
lines = []

if plotting:
    ax_a, ax_b = plt.subplots(1, 2, figsize=(16, 7), facecolor='w')[1]
    plt.show(block=False)

def get_distance(points, slope, intercept):
    """ return the distance for each point to the parametrised line """
    pos = np.array((0, intercept))  # origin
    dir = np.array((1, slope))  # line gradient
    # element-wise cross product of each points origin offset with the gradient
    c = np.cross(dir, pos - points, axisb=-1)
    return np.abs(c) / np.linalg.norm(dir)

def get_inliers(points, slope, intercept):
    """ return a numpy boolean array for each point (True if within 'inlier_dist' of the line, else False). """
    return get_distance(points, slope, intercept) <= inlier_dist


def find_best_params(points):
    """ find the best params to describe a detected line using the RANSAC algorithm """
    best_count = 0
    best_params = (0, 0)

    xs, ys = points.T

    # randomly sample points to define a line and remember the one with the most inliers
    for _ in xrange(sample_count):
        if (len(xs)==0):
            print("warn: The wall couldn't be found!")
            continue
        ind = np.random.randint(0, len(xs), 2)
        x_a, x_b = xs[ind]
        if x_a == x_b:
            continue  # avoid division by 0

        y_a, y_b = ys[ind].astype(np.float64)

        slope = (y_b - y_a) / (x_b - x_a)
        intercept = y_a - x_a * slope

        inlier = get_inliers(points, slope, intercept)
        inl_count = np.sum(inlier)
        if inl_count > best_count:
            best_count = inl_count
            best_params = (slope, intercept)

    # the set of points within inlier distance of the best line we found
    inlier_points = points[np.where(get_inliers(points, *best_params))]

    # perform a linear regression on the selected inlier points
    # slope, intercept, _, _, _ = stats.linregress(*inlier_points.T)
    slope, intercept = best_params

    return slope, intercept

def scan_callback(scan_msg):
    global wall1_angle, wall2_angle

    radius = np.asarray(scan_msg.ranges)
    angles = np.arange(scan_msg.angle_min, scan_msg.angle_max + scan_msg.angle_increment / 2, scan_msg.angle_increment)
    angles = angles - np.pi/2 # rotate by 90 degrees to get axis correct
    angles[angles < 0] = (2 * np.pi) + angles[angles < 0]

    mask_fin = np.isfinite(radius)  # only consider finite radii

    # Try to fit the lines with ransac and get the distance

    angle_spread_method1 = np.pi / 4  # size of cone to consider

    # Find the front wall
    mask_angle1 = np.logical_and((wall1_angle + angle_spread_method1) > angles, angles > (wall1_angle - angle_spread_method1))
    mask_front = np.logical_and(mask_fin, mask_angle1)

    # Find the right wall
    mask_angle2 = np.logical_and((wall2_angle + angle_spread_method1) > angles, angles > (wall2_angle - angle_spread_method1))
    mask_right = np.logical_and(mask_fin, mask_angle2)

    masked_angles_right = angles[mask_right]
    masked_radius_right = radius[mask_right]
    masked_angles_front = angles[mask_front]
    masked_radius_front = radius[mask_front]
    # calculate coordinates of our masked values
    x = np.cos(masked_angles_right) * masked_radius_right
    y = np.sin(masked_angles_right) * masked_radius_right
    points_right = np.column_stack((x, y))

    x = np.cos(masked_angles_front) * masked_radius_front
    y = np.sin(masked_angles_front) * masked_radius_front
    points_front = np.column_stack((x, y))

    slope1, intercept1 = find_best_params(points_front)  # detect a line in these coordinates
    slope2, intercept2 = find_best_params(points_right)

    wall1_dist_method1 = abs(intercept1)/pow(pow(slope1,2)+1.0,0.5)  # shortest distance from current position to the wall
    wall2_dist_method1 = abs(intercept2)/pow(pow(slope2,2)+1.0,0.5)

    if len(lines) == 0:
    	print "Pos1"
    	line = LineInfo(slope1, intercept1, wall1_dist_method1, slope2, intercept2, wall2_dist_method1, scan_msg.header.stamp)
    	lines.append(line)
    	pub_steering.publish(steering_angle)
    	pub_speed.publish(-150)
    if len(lines) == 1:
    	time_diff = scan_msg.header.stamp - lines[0].stamp
    	if time_diff.secs > 1:
    		print "Pos2"
    		pub_speed.publish(0)
    		line = LineInfo(slope1, intercept1, wall1_dist_method1, slope2, intercept2, wall2_dist_method1, scan_msg.header.stamp)
    		lines.append(line)
    		pub_speed.publish(-150)
    if len(lines) == 2:
    	time_diff = scan_msg.header.stamp - lines[1].stamp
    	if time_diff.secs > 1:
    		print "Pos3"
    		pub_speed.publish(0)
    		line = LineInfo(slope1, intercept1, wall1_dist_method1, slope2, intercept2, wall2_dist_method1, scan_msg.header.stamp)
    		x0 = Symbol('x0')
    		y0 = Symbol('y0')
    		r = Symbol('r')
    		eq1 = (lines[0].wall_dist2 - x0)**2 + (lines[0].wall_dist1 - y0)**2 - r**2
    		eq2 = (lines[1].wall_dist2 - x0)**2 + (lines[1].wall_dist1 - y0)**2 - r**2
    		eq3 = (line.wall_dist2 - x0)**2 + (line.wall_dist1 - y0)**2 - r**2
    		result = nonlinsolve([eq1, eq2, eq3], [x0, y0, r])
    		radius = float(result.args[0][2])
    		print "Radius: ", radius
    		angle = np.arctan(0.26/radius)
    		print "Angle: ", angle
    		rospy.signal_shutdown('stop')
    if plotting:
        ax_a.cla()
        ax_a.set_title('Scatter plot of laser scan data')

        # plot inliers detected by ransac
        inlier_mask1 = get_inliers(points_right, slope1, intercept1)
        inlier_mask2 = get_inliers(points_front, slope2, intercept2)
        #ax_a.scatter(*points_right[np.where(inlier_mask1)].T, color='r')
        ax_a.scatter(*points_front.T, color='g')
        #ax_a.scatter(*points_front[np.where(inlier_mask2)].T, color='g')
        #ax_a.scatter(*points_right[np.where(np.logical_not(inlier_mask1))].T, color='b')
        #ax_a.scatter(*points_front[np.where(np.logical_not(inlier_mask2))].T, color='b')

        #x = np.cos(angles) * radius
       # y = np.sin(angles) * radius
       # ax_a.scatter(x, y, color='k')

        # plot any filtered points
        inv_mask = np.logical_not(np.logical_and(mask_right, mask_front))
        other_x = np.cos(angles[inv_mask]) * radius[inv_mask]
        other_y = np.sin(angles[inv_mask]) * radius[inv_mask]
        #ax_a.scatter(other_x, other_y, color='k')

        # how many meters to plot in each direction
        plt_window = 3.5
        ax_a.set_xlim([-plt_window, plt_window])
        ax_a.set_ylim([-plt_window, plt_window])

        line_x = np.linspace(-plt_window, plt_window, 10)
        line_y = intercept1 + line_x * slope1
        ax_a.plot(line_x, line_y, color='b')

        line2_x = np.linspace(-plt_window, plt_window, 10)
        line2_y = intercept2 + line2_x * slope2
        ax_a.plot(line2_x, line2_y, color='g')

        # draw coordinate system
        ax_a.axvline(0, color='k')
        ax_a.axhline(0, color='k')
        plt.show(block=False)
        rospy.sleep(0.1)  # sleep to avoid threading issues with plotting

def main(args):
    rospy.init_node("angle_calibration1")
    rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=1)
    #rospy.Subscriber("/steering", UInt8, steering_callback, queue_size=1)
    if plotting:
        plt.show()  # block until plots are closed
    else:
        rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
