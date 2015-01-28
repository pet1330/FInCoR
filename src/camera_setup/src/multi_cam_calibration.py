#!/usr/bin/python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf

class calibrate_camera:

    def __init__(self):
        self.bridge = CvBridge()
        self.left_image = rospy.Subscriber('/left/rgb/image_color', Image, self.left_callback)
        self.right_image = rospy.Subscriber('/right/rgb/image_color', Image, self.right_callback)
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        square_size = 0.05
        self.pattern_size = (6, 8)
        self.pattern_points = np.zeros( (np.prod(self.pattern_size), 3), np.float32)
        self.pattern_points[:,:2] = np.indices(self.pattern_size).T.reshape(-1, 2)
        self.pattern_points *= square_size
    
    def find_tf(self, ros_image_msg, invert):
        try:
            #Colour Image
            cv_image = self.bridge.imgmsg_to_cv2(ros_image_msg, "bgr8")
        except CvBridgeError, e:
            print e
        cv_image = cv2.cvtColor(cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY), cv2.COLOR_GRAY2RGB)
        found, corners = cv2.findCirclesGrid(cv_image, self.pattern_size,flags = cv2.CALIB_CB_SYMMETRIC_GRID)
        obj_points = []
        img_points = []
        
        if found:
            img_points.append(corners.reshape(-1, 2))
            if invert:
                img_points = np.flipud(np.fliplr(img_points))
            obj_points.append(self.pattern_points)
            camera_matrix=np.array([525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]).reshape(3,3)
            dist_coefs=np.empty((5,1))
            _, rVec, tVec = cv2.solvePnP(np.array(obj_points),np.array(img_points),camera_matrix,dist_coefs)
            Rt = cv2.Rodrigues(rVec)[0] 
            rot_hom = np.eye(4)
            rot_hom[:3,:3]=Rt
            q = tf.transformations.quaternion_from_matrix(rot_hom)
            
            if invert:
                self.br.sendTransform((tVec[0],tVec[1] , tVec[2]),q,rospy.Time.now(), "/marker_right", "/right/right_rgb_optical_frame")
                try:
                    (transr,rotr) = self.listener.lookupTransform('/marker_right', '/right/right_link', rospy.Time(0))
                    self.br.sendTransform(transr,rotr,rospy.Time.now(), '/right/right_link', "/base")
                    print "no error right"
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass
                
            else:
                self.br.sendTransform((tVec[0],tVec[1] , tVec[2]),q,rospy.Time.now(), "/marker_left", "/left/left_rgb_optical_frame")
                try:
                    (transl,rotl) = self.listener.lookupTransform('/marker_left', '/left/left_link', rospy.Time(0))
                    self.br.sendTransform(transl,rotl,rospy.Time.now(), '/left/left_link', "/base")
                    print "no error left"

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass

    def left_callback(self,left):
        self.find_tf(left,invert = False)

    def right_callback(self,right):
        self.find_tf(right,invert = True)

def main():
    rospy.init_node('calibrate_camera', anonymous=True)
    calibrate_camera()
    rospy.spin()

if __name__ == '__main__':
    main()
