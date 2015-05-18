#!/usr/bin/python
import cv2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import numpy as np
import rosparam
import rospy
from sensor_msgs.msg import Image
import sys
import tf
import rospkg

class calibrate_camera:

    def __init__(self):
        
        try:
        	self.no_frames = rospy.get_param('/camera_calibration/frames');
        except:
        	self.no_frames = 1000
        print self.no_frames
        self.pattern_size = (6, 8)
        
        self.pattern_points = np.zeros((np.prod(self.pattern_size), 3), np.float32)
        self.pattern_points[:, :2] = np.indices(self.pattern_size).T.reshape(-1, 2)
        self.pattern_points[:, 0] = -self.pattern_points[:, 0]
        self.pattern_points *= 0.05
        self.count = [1, 1]
        self.left_midXYZ = [0, 0, 0]
        self.left_midQ = [0, 0, 0, 0]
        self.right_midXYZ = [0, 0, 0]
        self.right_midQ = [0, 0, 0, 0]
        
        self.final_left_xyz = [0, 0, 0]
        self.final_left_q = [0, 0, 0, 1]
        self.final_right_xyz = [0, 0, 0]
        self.final_right_q = [0, 0, 0, 1]
        self.still_calibrating = True
        self.bridge = CvBridge()
        self.left_image = rospy.Subscriber('/left/rgb/image_color', Image, self.left_callback)
        self.right_image = rospy.Subscriber('/right/rgb/image_color', Image, self.right_callback)
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.pt = {'previous_transform': {'left': {'orientation': [0.0, 0.0, 0.0, 0.0],
                                 'position': [0.0, 0.0, 0.0]},
                        'right': {'orientation': [0.0, 0.0, 0.0, 0.0],
                                  'position': [0.0, 0.0, 0.0]}}}
        
        self.rospack = rospkg.RosPack()
        try:
            self.pt = rosparam.load_file(self.rospack.get_path('camera_setup') + '/config/previous_transform.yaml')[0][0]
        except:
            self.still_calibrating = True

        if rospy.get_param('~quick'):
            self.final_left_xyz = self.pt['previous_transform']['left']['position']
            self.final_left_q = self.pt['previous_transform']['left']['orientation']
            self.final_right_xyz = self.pt['previous_transform']['right']['position']
            self.final_right_q = self.pt['previous_transform']['right']['orientation']
            self.still_calibrating = False
            self.publish_feedback()
        
    def find_tf(self, ros_image_msg, invert):
        try:
            #Colour Image
            cv_image = self.bridge.imgmsg_to_cv2(ros_image_msg, "bgr8")
        except CvBridgeError, e:
            print e
        cv_image = cv2.cvtColor(cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY), cv2.COLOR_GRAY2RGB)
        found, corners = cv2.findCirclesGrid(cv_image, self.pattern_size, flags=cv2.CALIB_CB_SYMMETRIC_GRID)
        obj_points = []
        img_points = []
        
        if found:
            img_points.append(corners.reshape(-1, 2))
            if invert:
                img_points = np.flipud(np.fliplr(img_points))
            obj_points.append(self.pattern_points)
            #print obj_points
            camera_matrix = np.array([525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]).reshape(3, 3)
            dist_coefs = np.empty((5, 1))
            _, rVec, tVec = cv2.solvePnP(np.array(obj_points), np.array(img_points), camera_matrix, dist_coefs)
            Rt = cv2.Rodrigues(rVec)[0] 
            rot_hom = np.eye(4)
            rot_hom[:3, :3] = Rt
            q = tf.transformations.quaternion_from_matrix(rot_hom)
            
            if invert:
                self.right_midXYZ = np.vstack((self.right_midXYZ, tVec.reshape(1, -1)[0]))
                self.right_midQ = np.vstack((self.right_midQ, q))
                self.count[1] += 1
                self.update_progress()
                if self.count[1] == self.no_frames:
                    self.final_right_xyz = np.median(self.right_midXYZ[1:], axis=0)
                    median_Q = np.median(self.right_midQ[1:], axis=0)
                    self.final_right_q = median_Q / np.sum(median_Q)
            else:
                self.left_midXYZ = np.vstack((self.left_midXYZ, tVec.reshape(1, -1)[0]))
                self.left_midQ = np.vstack((self.left_midQ, q))
                self.count[0] += 1
                #print self.count
                self.update_progress()
                if self.count[0] == self.no_frames:
                    self.final_left_xyz = np.median(self.left_midXYZ[1:], axis=0)
                    median_Q = np.median(self.left_midQ[1:], axis=0)
                    self.final_left_q = median_Q / np.sum(median_Q)
            
            if self.count[0] > self.no_frames and self.count[1] > self.no_frames and self.still_calibrating:
                self.still_calibrating = False
                
                print "Successfully Calibrated over %d frames, " % self.no_frames

                self.pt['previous_transform']['left']['position'] = self.final_left_xyz.tolist()
                self.pt['previous_transform']['right']['position'] = self.final_right_xyz.tolist()
                self.pt['previous_transform']['left']['orientation'] = self.final_left_q.tolist()
                self.pt['previous_transform']['right']['orientation'] = self.final_right_q.tolist()

                print self.pt
                rospy.set_param('calibrate_camera',self.pt)
                rosparam.dump_params(self.rospack.get_path('camera_setup') + '/config/previous_transform.yaml', 'calibrate_camera', verbose=False)
                self.publish_feedback()

    def update_progress(self):
        progress = (float(min(self.count)))
        percent = (progress / self.no_frames)
        if percent <= 1.0:
            sys.stdout.write('\r')
            sys.stdout.write("[%-70s] %.1f%%" % ('#' * int(percent * 70), percent * 100.00))
            sys.stdout.flush()
        else:
            print ""

    def publish_feedback(self):
        print "Starting Static Transform Broadcaster on:"
        print "+" + "-" * 5 + "+" + "-" * 12  + "+" + "-" * 12  + "+"
        print "|     | %-10s | %-10s |" % ("  Left:  ", "  Right:  ")
        print "+" + "-" * 5 + "+" + "-" * 12  + "+" + "-" * 12  + "+"
        print "| %-3s | %-10f | %-10f |" % ("X:", self.final_left_xyz[0], self.final_right_xyz[0])
        print "| %-3s | %-10f | %-10f |" % ("Y:", self.final_left_xyz[1], self.final_right_xyz[1])
        print "| %-3s | %-10f | %-10f |" % ("Z:", self.final_left_xyz[2], self.final_right_xyz[2])
        for index in range(len(self.final_left_q)):
            print "| %-3s | %-10f | %-10f |" % ("Q" + str(index + 1) + ":", self.final_left_q[index], self.final_right_q[index])
        print "+" + "-" * 5 + "+" + "-" * 12  + "+" + "-" * 12  + "+"


    def left_publish(self):
        self.br.sendTransform((self.final_left_xyz[0], self.final_left_xyz[1], self.final_left_xyz[2]), self.final_left_q, rospy.Time.now(), "/marker_left", "/left/left_rgb_optical_frame")
        try:
            (transl, rotl) = self.listener.lookupTransform('/marker_left', '/left/left_link', rospy.Time(0))
            self.br.sendTransform(transl, rotl, rospy.Time.now(), '/left/left_link', "/base")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
    
    def right_publish(self):
        self.br.sendTransform((self.final_right_xyz[0], self.final_right_xyz[1], self.final_right_xyz[2]), self.final_right_q, rospy.Time.now(), "/marker_right", "/right/right_rgb_optical_frame")
        try:
            (transr, rotr) = self.listener.lookupTransform('/marker_right', '/right/right_link', rospy.Time(0))
            self.br.sendTransform(transr, rotr, rospy.Time.now(), '/right/right_link', "/base")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def left_callback(self, left):
        if self.still_calibrating:
            self.find_tf(left, invert=False)
        else:
            self.left_publish()
        

    def right_callback(self, right):
        if self.still_calibrating:
            self.find_tf(right, invert=True)
        else:
            self.right_publish()


def main():
    rospy.init_node('calibrate_camera')
    calibrate_camera()
    rospy.spin()

if __name__ == '__main__':
    main()

