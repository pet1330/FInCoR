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
import message_filters
from camera_calibration.approxsync import ApproximateSynchronizer


class calibrate_camera:
    
  def __init__(self):

    self.image_pub = rospy.Publisher("image_topic_2",Image)    
    self.bridge = CvBridge()
    self.left_image = message_filters.Subscriber('/left/rgb/image_color', Image)
    self.right_image = message_filters.Subscriber('/right/rgb/image_color', Image)
    ts = ApproximateSynchronizer(0.1,[self.left_image,self.right_image],10)
    ts.registerCallback(self.callback)
    # Create a window
    cv2.namedWindow('image',cv2.WINDOW_NORMAL)

  def callback(self,left,right):
    try:
      #Colour Image
      cv_left_image = self.bridge.imgmsg_to_cv2(left, "bgr8")
      cv_right_image = self.bridge.imgmsg_to_cv2(right, "bgr8")
    except CvBridgeError, e:
      print e

    cv_left_image = cv2.cvtColor(cv2.cvtColor(cv_left_image, cv2.COLOR_RGB2GRAY), cv2.COLOR_GRAY2RGB)
    shape = (10, 7)
    [isFound, centers] = cv2.findCirclesGrid(cv_left_image, shape,flags = cv2.CALIB_CB_SYMMETRIC_GRID)
    #    print str(isFound) + " - CALIB_CB_SYMMETRIC_GRID"
    cv2.drawChessboardCorners(cv_left_image,shape,centers,isFound)
    
    cv_right_image = cv2.cvtColor(cv2.cvtColor(cv_right_image, cv2.COLOR_RGB2GRAY), cv2.COLOR_GRAY2RGB)
    [isFound2, centers2] = cv2.findCirclesGrid(cv_right_image, shape,flags = cv2.CALIB_CB_SYMMETRIC_GRID)
    cv2.drawChessboardCorners(cv_right_image,shape,centers2,isFound2)

    #print cv2.solvePnP(centers, centers, cameraMatrix, distCoeffs)

    # Combine original image and mask
    h, w = cv_left_image.shape[:2]
    vis = np.zeros((h, w * 2), np.uint8)
    vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
    vis[:h, :w] = cv_left_image # cv2.cvtColor(cv_left_image, cv2.COLOR_GRAY2BGR)
    vis[:h, w:w * 2] = cv_right_image # cv2.cvtColor(cv_right_image, cv2.COLOR_GRAY2BGR)
    # Show image
    cv2.imshow('image', vis)
    k = cv2.waitKey(1) & 0xFF
    print centers
    print "#############################################"

    #print two + " - CALIB_CB_CLUSTERING"
    #cv2.imshow('left_image', cv_left_image)
    #k = cv2.waitKey(1) & 0xFF
    #cv2.imshow('right_image', cv_right_image)
    k = cv2.waitKey(1) & 0xFF


    # Publish new image back to the ROS system
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_left_image, "bgr8"))
    except CvBridgeError, e:
      print e

def main(args):
  rospy.init_node('calibrate_camera', anonymous=True)
  ic = calibrate_camera()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
