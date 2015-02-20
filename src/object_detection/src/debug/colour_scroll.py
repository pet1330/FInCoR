#!/usr/bin/python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from yaml import load, dump

def nothing(x):
    pass

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)    
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)
    # Create a window
    cv2.namedWindow('image',cv2.WINDOW_NORMAL)
    # Create trackbars
    cv2.createTrackbar('H_low', 'image', 16, 180, nothing)
    cv2.createTrackbar('H_high', 'image', 25, 180, nothing)
    cv2.createTrackbar('S_low', 'image', 40, 255, nothing)
    cv2.createTrackbar('S_high', 'image', 150, 255, nothing)
    cv2.createTrackbar('V_low', 'image', 97, 255, nothing)
    cv2.createTrackbar('V_high', 'image', 172, 255, nothing)
  
  def colourFilter(self,HSV_image):
    # Get slider values
    h = cv2.getTrackbarPos('H_low', 'image'), cv2.getTrackbarPos('H_high', 'image')
    s = cv2.getTrackbarPos('S_low', 'image'), cv2.getTrackbarPos('S_high', 'image')
    v = cv2.getTrackbarPos('V_low', 'image'), cv2.getTrackbarPos('V_high', 'image')

    # Thresholds
    lower = np.array([h[0], s[0], v[0]])
    upper = np.array([h[1], s[1], v[1]])
    # Calculate mask using thresholds
    mask = cv2.inRange(HSV_image, lower, upper)
    return mask


  def callback(self,data):
    try:
      #Colour Image
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e
    img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = self.colourFilter(img)
    # Combine original image and mask
    h, w = img.shape[:2]
    vis = np.zeros((h, w * 2), np.uint8)
    vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
    vis[:h, :w] = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    vis[:h, w:w * 2] = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    # Show image
    cv2.imshow('image', vis)
    k = cv2.waitKey(1) & 0xFF

    # Publish new image back to the ROS system
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
    except CvBridgeError, e:
      print e

#==============================================================

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)