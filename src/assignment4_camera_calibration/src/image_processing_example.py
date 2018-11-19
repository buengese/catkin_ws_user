#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt

# from __future__ import print_function

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/bin_img2",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback, queue_size=1)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # crop the image to remove reflections on the car
    cv_image_cropped = cv_image[0:420, 0:640]

    # make it gray
    gray=cv2.cvtColor(cv_image_cropped, cv2.COLOR_BGR2GRAY)

    bi_gray_max = 255
    bi_gray_min = 245
    ret,thresh1=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);

    # publish the final image
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8"))
    except CvBridgeError as e:
      print(e)

        ##Getting the points from image##
    image_points = np.zeros((6,2));

    for i in range (0,220):
        for j in range(0,320):
                  if (thresh1[i,j] >= 200):
                      image_points[0,0]=j
                      image_points[0,1]=i



    for i in range (220,300):
        for j in range (0,320):
               if (thresh1[i,j] >= 200):
                      image_points[1,0]=j
                      image_points[1,1]=i



    for i in range(300,420):
       for j in range(0,320):
                 if (thresh1[i,j] >= 200):
                      image_points[2,0]=j
                      image_points[2,1]=i



    for i in range(0,220):
       for j in range(320,640):
                 if (thresh1[i,j] >= 200):
                      image_points[3,0]=j
                      image_points[3,1]=i



    for i in range(220,300):
        for j in range(320,640):
                  if (thresh1[i,j] >= 200):
                      image_points[4,0]=j
                      image_points[4,1]=i


    for i in range (300,420):
        for j in range(320,640):
               if (thresh1[i,j] >= 200):
                      image_points[5,0]=j
                      image_points[5,1]=i


    print 'points: \n', image_points

    fx = 614.1699
    fy = 614.9002
    cx = 329.9491
    cy = 237.2788
    camera_mat = np.zeros((3,3,1))
    camera_mat[:,:,0] = np.array([[fx, 0, cx],
                                  [0, fy, cy],
                                  [0, 0, 1]])
    k1 = 0.1115
    k2 = -0.1089
    p1 = 0
    p2 = 0
    dist_coeffs = np.zeros((4,1))
    dist_coeffs[:,0] = np.array([[k1, k2, p1, p2]])
    # far to close, left to right (order of discovery) in cm
    obj_points = np.zeros((6,3,1))
    obj_points[:,:,0] = np.array([[0, 0, 0],
                                  [21.8, 0, 0],
                                  [0, 30.0, 0],
                                  [22.2, 30.0, 0],
                                  [0, 60.0, 0],
                                  [22.0, 60.0, 0]])
    retval, rvec, tvec = cv2.solvePnP(obj_points, image_points,camera_mat, dist_coeffs)
    print 'rvec \n' , rvec
    print 'tvec \n' , tvec


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
