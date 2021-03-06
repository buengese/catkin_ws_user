#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import math
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
    self.image_pub = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)

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

    # turn image black
    bi_gray_max = 255
    bi_gray_min = 240
    ret,thresh1=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);

    # publish the black and white image
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8"))
    except CvBridgeError as e:
      print(e)

    image_points = np.zeros((6,2));

    # Each loop searches for one point because our points are very of center in the area search by each loop is very different
    # top left point
    for i in range (0,220):
        for j in range(0,320):
                  if (thresh1[i,j] >= 200):
                      image_points[0,0]=j #
                      image_points[0,1]=i

    # middle left
    for i in range (220,300):
        for j in range (0,320):
               if (thresh1[i,j] >= 200):
                      image_points[1,0]=j
                      image_points[1,1]=i


    # bottem left
    for i in range(300,420):
       for j in range(0,320):
                 if (thresh1[i,j] >= 200):
                      image_points[2,0]=j
                      image_points[2,1]=i

    # top right                  
    for i in range(0,220):
       for j in range(320,640):
                 if (thresh1[i,j] >= 200):
                      image_points[3,0]=j
                      image_points[3,1]=i

    # middle right                  
    for i in range(220,300):
        for j in range(320,640):
                  if (thresh1[i,j] >= 200):
                      image_points[4,0]=j
                      image_points[4,1]=i

    # bottem right
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

    obj_points = np.zeros((6,3,1))
    # left far to close, right far to close
    obj_points[:,:,0] = np.array([[0, 0, 0],
                                  [0, 30.0, 0],
                                  [0, 58.3, 0],
                                  [41.7, 0, 0],
                                  [40.0, 30.0, 0],
                                  [40.0, 58.4, 0]])
    retval, rvec, tvec = cv2.solvePnP(obj_points, image_points,camera_mat, dist_coeffs)
    print 'rvec \n' , rvec
    print 'tvec \n' , tvec

    rmat = cv2.Rodrigues(rvec)[0]
    print 'rmat \n' , rmat
    inv_rmat = rmat.transpose()
    print 'inv_rmat \n' , inv_rmat
    inv_rmat_ = np.negative(inv_rmat)
    inv_tvec = inv_rmat_.dot(tvec)
    print 'inv_tvec \n' , inv_tvec
    sy = math.sqrt(rmat[0,0] * rmat[0,0] +  rmat[1,0] * rmat[1,0]);
    singular = sy < 1e-6; # If
    if (~singular):
         roll = math.atan2(-rmat[2,1] , rmat[2,2]);
         pitch = math.atan2(-rmat[2,0], sy);
         yaw = math.atan2(rmat[1,0], rmat[0,0]);
    else:
         roll = math.atan2(-rmat[1,2], rmat[1,1]);
         pitch = math.atan2(-rmat[2,0], sy);
         yaw = 0;

    print 'position (x, y, z)', inv_tvec[0]/100, inv_tvec[1]/100, inv_tvec[2]/100
    print 'rotation (yaw, pitch, roll)', yaw, pitch, roll


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
