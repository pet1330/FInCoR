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
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/right/rgb/image_color",Image,self.callback)
    cv2.namedWindow('image',cv2.WINDOW_NORMAL)

  def callback(self,data):
    try:
      #Colour Image
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv2.imwrite( "/home/peter/Desktop/data/owncloud/preditor_training/" + str(rospy.Time.now()) + ".jpg", cv_image)
      cv2.imshow('image', cv_image)
      k = cv2.waitKey(1) & 0xFF
    except CvBridgeError, e:
      print e

def main(args):
  rospy.init_node('saveImage', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)