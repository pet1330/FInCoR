#!/usr/bin/python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import thread
import time
import yaml

class colourFilter:
    

  def __init__(self):
    

    pink = rospy.get_param("~colour_thresh")
    print pink["brown"][0][0]
    
    self.image_pub = rospy.Publisher("image_topic_2",Image)    
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)
    # Create a window
    cv2.namedWindow('image',cv2.WINDOW_NORMAL)
    
  def colourFilter(self,HSV_image):
    # Get slider values
    hsv_low = (0,0,0)
    hsv_high = (180,255,255)

    # Thresholds
    lower = np.array([hsv_low[0], hsv_low[1], hsv_low[2]])
    upper = np.array([hsv_high[0], hsv_high[1], hsv_high[2]])
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
#res = cv2.bitwise_and(frame,frame, mask= mask)
#==============================================================

def main(args):
  rospy.init_node('colourFilter', anonymous=True)
  ic = colourFilter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)