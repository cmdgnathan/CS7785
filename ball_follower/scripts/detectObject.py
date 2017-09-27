#!/usr/bin/env python
import roslib
roslib.load_manifest('ball_follower')
import sys
import rospy
import cv2
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class detectObject:

  def __init__(self):


    self.image_pub = rospy.Publisher("/image/compressed", CompressedImage, queue_size=1)    
    self.point_pub = rospy.Publisher("/control/angle/setpoint", Point, queue_size=1)

    #self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.img_cb)
    self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.img_cb)    
    
    self.color_lower_sub = rospy.Subscriber("/parameter/color_lower", Point, self.color_lower_cb)    
    self.color_upper_sub = rospy.Subscriber("/parameter/color_upper", Point, self.color_upper_cb)    


    self.bridge = CvBridge()

    # Images
    #self.img_bgr8 
    #self.img_grey 

    # Color Data
    self.color_box_width = 10
    self.color_range = 40
    self.color = (0,0,0)
    self.color_selected = False

    self.color_upper = (0,0,0)
    self.color_lower = (0,0,0)

    self.lower_history = []
    self.upper_history = []

    # Control Variables
    self.x_pct = 0.5
    self.y_pct = 0.5
    self.fill_pct = 0.0


    # Debug
    self.debug = False;

    return


  def color_upper_cb(self, data):
    self.upper_history.append( (data.x,data.y,data.z) )
    return

  def color_lower_cb(self, data):
    self.lower_history.append( (data.x,data.y,data.z) )
    return



  def img_cb(self,data):

    

    try:
      self.img_bgr8 = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
      self.img_hsv8 = cv2.cvtColor( self.img_bgr8, cv2.COLOR_BGR2HSV )
      self.img_grey = cv2.cvtColor( self.img_bgr8, cv2.COLOR_BGR2GRAY )

      #self.img_mask = np.zeros( self.img_grey.shape[:2], dtype="uint8" )
    except CvBridgeError as e:
      print(e)


    if min(len(self.lower_history),len(self.upper_history))>0:

      ##############################################################################
      # Color Mask
      cumulative_mask = cv2.inRange( self.img_bgr8, self.lower_history[-1], self.upper_history[-1])

      for i in range(0,min(len(self.lower_history),len(self.upper_history))):
        mask = cv2.inRange( self.img_bgr8, self.lower_history[i], self.upper_history[i])
        cumulative_mask = cv2.bitwise_or(mask, cumulative_mask, cumulative_mask)
        
      ##############################################################################
      # Erosion / Dilation
      
      kernel = np.ones((5,5), np.uint8)

      
      #img_erosion = cv2.erode(cumulative_mask, kernel, iterations = 1)
      #img_dilation = cv2.dilate(img_erosion, kernel, iterations = 3)
      
      #self.img_down = cv2.resize(img_erosion, None, fx=0.1, fy=0.1, interpolation = cv2.INTER_NEAREST)
      #img_center = self.img_down

      img_center = cumulative_mask[::4,::4]

      h = 0
      w = 0
      n = 0
      height, width = img_center.shape[:2]


      for i in range(height):
        for j in range(width):
          if img_center[i,j]:
            h+=i
            w+=j
            n+=1


      h /= max(n,1)
      w /= max(n,1)

      if self.debug:
        print height, width
        print self.lower_history, self.upper_history
        print img_center.shape  
        print h, w
        print "Ball:", h, w
      
      print "X:", self.x_pct, "Y:", self.y_pct, "Fill:", self.fill_pct
      

      self.y_pct = h / float(height)
      self.x_pct = w / float(width)

      self.fill_pct = n / float(height*width)

    return




def main(args):
  ic = detectObject()
  rospy.init_node('detectObject', anonymous=True)

  try:
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      ic.point_pub.publish( Point(ic.x_pct, ic.y_pct, ic.fill_pct) )
      rate.sleep()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
      main(sys.argv)
    except rospy.ROSInterruptException:
      pass