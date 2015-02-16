#!/usr/bin/python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import util
import CMT
from numpy import empty, nan
import os
import sys
import time


class object_track:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/right/rgb/image_color",Image,self.callback)
    cv2.namedWindow('image',cv2.WINDOW_NORMAL)
    self.CMT = CMT.CMT()
    self.firstFrame = True;



  def callback(self,data):
    try:
      #Colour Image
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

      if self.firstFrame:
        im_gray0 = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        im_draw = np.copy(cv_image)

        # Get rectangle input from user
        (tl, br) = util.get_rect(im_draw)
        print 'using', tl, br, 'as initial box'
        self.CMT.initialise(im_gray0, tl, br)
        self.firstFrame = False;

      else:

        im_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        im_draw = np.copy(cv_image)

        tic = time.time()
        self.CMT.process_frame(im_gray)
        toc = time.time()

        # Display results
        # Draw updated estimate
        if self.CMT.has_result:

          cv2.line(im_draw, self.CMT.tl, self.CMT.tr, (255, 0, 0), 4)
          cv2.line(im_draw, self.CMT.tr, self.CMT.br, (255, 0, 0), 4)
          cv2.line(im_draw, self.CMT.br, self.CMT.bl, (255, 0, 0), 4)
          cv2.line(im_draw, self.CMT.bl, self.CMT.tl, (255, 0, 0), 4)

        util.draw_keypoints(self.CMT.tracked_keypoints, im_draw, (255, 255, 255))
        # this is from simplescale
        util.draw_keypoints(self.CMT.votes[:, :2], im_draw)  # blue
        util.draw_keypoints(self.CMT.outliers[:, :2], im_draw, (0, 0, 255))
        cv2.imshow('image', im_draw)
  
        # Check key input
        k = cv2.waitKey(1) & 0xFF
        key = chr(k & 255)
        #print '{5:04d}: center: {0:.2f},{1:.2f} scale: {2:.2f}, active: {3:03d}, {4:04.0f}ms'.format(self.CMT.center[0], self.CMT.center[1], self.CMT.scale_estimate, self.CMT.active_keypoints.shape[0], 1000 * (toc - tic), -999)

    except CvBridgeError, e:
      print e

def main(args):
  rospy.init_node('object_track', anonymous=True)
  object_track()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)