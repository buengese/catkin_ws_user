#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sklearn import linear_model

# from __future__ import print_function

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)
    self.output_pub = rospy.Publisher("/image_processing/lines", Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback, queue_size=1)

  # Let's cheat a little with some fancy stuff from cv2
  def find_segments(self, img):
    (_, contours, _) = cv2.findContours(img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
    return contours[0]

  # Using ransac from sklearn library
  def ransac(self, contour):
    ransac = linear_model.RANSACRegressor()

    x = []
    y = []
    for val in contour:
        x.append([val[0][1]])
        y.append([val[0][0]])

    ransac.fit(x, y)
    b = ransac.estimator_.intercept_
    m = ransac.estimator_.coef_

    return m, b

  # Returns start and end points for our line
  def get_line_points(self, m, b, width):
    x1 = 0
    x2 = width
    y1 = (x1 * m + b)
    y2 = (x2 * m + b)
    return ((y1,x1),(y2,x2))

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #y_end = np.shape(cv_image)[0]
    #y_start = np.shape(cv_image)[0] * 0.5
    #cv_image = cv_image[int(y_start):int(y_end), :]

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0,0,250]) # 0, 40, 150 / 0, 0, 0 / 0, 0, 250
    upper_white = np.array([30,150,255]) # 18, 80, 255 / 255, 60, 255 / 50, 200, 255

    mask = cv2.inRange(hsv, lower_white, upper_white)
    res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    try:
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

    seg1 = self.find_segments(mask)

    m1, b1 = self.ransac(seg1)

    line1 = self.get_line_points(m1, b1, cv_image.shape[1])

    cv2.line(cv_image, line1[0], line1[1], (255, 0, 0), 5)
    try:
      self.output_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

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
