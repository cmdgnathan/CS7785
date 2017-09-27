#!/usr/bin/env python
import roslib
roslib.load_manifest('ball_follower')
import sys
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class select_color:

  def __init__(self):


    self.color_lower_pub = rospy.Publisher("/parameter/color_lower", Point, queue_size=1)
    self.color_upper_pub = rospy.Publisher("/parameter/color_upper", Point, queue_size=1)

    #self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.img_cb)
    self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.img_cb)    

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


    return


  def mouse_cb(self,event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK:

      #if x-10<0:
      #  x += abs(x-10)
      #elif x+10>

      x1 = x-self.color_box_width
      x2 = x+self.color_box_width
      y1 = y-self.color_box_width
      y2 = y+self.color_box_width

      self.color_box = self.img_bgr8[y1:y2,x1:x2]

      #print self.color_box


      height, width = self.color_box.shape[:2]

      channel = np.zeros( (3,height*width), dtype="uint8" )

      for h in range(0, height):
        for w in range(0, width):
          index = h*width+w
          for c in range(0, 3):
            channel[c][index] = self.color_box[h,w][c]
         
      self.color = [ int( np.median( channel[c] ) ) for c in range(0, 3) ]
      self.color_std = [ max(20, int( 2*np.std( channel[c] ) ) ) for c in range(0,3) ]

      self.color_lower = [ max(self.color[c]-self.color_std[c],0) for c in range(0,3)]
      self.color_upper = [ min(self.color[c]+self.color_std[c],255) for c in range(0,3)]

      self.color = np.array(self.color, dtype="uint8")
      self.color_lower = np.array(self.color_lower, dtype="uint8")
      self.color_upper = np.array(self.color_upper, dtype="uint8")

      print "HSV:"+str(self.color), self.color_std

      cv2.imshow("HSV:"+str(self.color), self.color_box)
      cv2.waitKey(1) 

      self.color_selected = True


      self.lower_history.append( self.color_lower )
      self.upper_history.append( self.color_upper )


      # Publish Parameters to Robot
      self.color_lower_pub.publish( Point(self.color_lower[0],self.color_lower[1],self.color_lower[2]) )
      self.color_upper_pub.publish( Point(self.color_upper[0],self.color_upper[1],self.color_upper[2]) )

    return



  def img_cb(self,data):

    # Name Color Selector
    cv2.namedWindow("Double-Click to Select Color")
    cv2.setMouseCallback("Double-Click to Select Color", self.mouse_cb)

    try:
      self.img_bgr8 = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
      self.img_hsv8 = cv2.cvtColor( self.img_bgr8, cv2.COLOR_BGR2HSV )
      self.img_grey = cv2.cvtColor( self.img_bgr8, cv2.COLOR_BGR2GRAY )

      #self.img_mask = np.zeros( self.img_grey.shape[:2], dtype="uint8" )
    except CvBridgeError as e:
      print(e)


    if self.color_selected:

      ##############################################################################
      # Color Mask
      cumulative_mask = cv2.inRange( self.img_bgr8, self.color_lower, self.color_upper)

      for i in range(0,len(self.lower_history)):
        mask = cv2.inRange( self.img_bgr8, self.lower_history[i], self.upper_history[i])
        cumulative_mask = cv2.bitwise_or(mask, cumulative_mask, cumulative_mask)

      ##############################################################################
      # Erosion / Dilation
      
      kernel = np.ones((5,5), np.uint8)

      
      img_erosion = cv2.erode(cumulative_mask, kernel, iterations = 1)
      img_dilation = cv2.dilate(img_erosion, kernel, iterations = 3)


      
      self.img_down = cv2.resize(img_erosion, None, fx=0.1, fy=0.1, interpolation = cv2.INTER_NEAREST)


      img_center = self.img_down


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

      self.y_pct = h / float(height)
      self.x_pct = w / float(width)

      self.fill_pct = n / float(height*width)


      #print "Ball:", h, w
      #print "X:", self.x_pct, "Y:", self.y_pct, "Fill:", self.fill_pct


      # Circle Track Point
      cv2.circle(img_center,(w,h),20,(255,0,0),2)


      # Image Views
      cv2.imshow("Recent Mask", mask)
      cv2.moveWindow("Recent Mask", 650, 0)

      cv2.imshow("Cumulative Mask", cumulative_mask)
      cv2.moveWindow("Cumulative Mask", 650, 500)

      cv2.imshow("Erosion", img_erosion)
      cv2.moveWindow("Erosion", 1300, 0)
      
      cv2.imshow("Dilation", img_dilation)
      cv2.moveWindow("Dilation", 1300, 500)
      
      cv2.imshow("Output", img_center)
      cv2.moveWindow("Output", 0, 500)      


    # Show Gray Image
    cv2.imshow("Double-Click to Select Color", self.img_grey)
    cv2.waitKey(1)



    return




def main(args):
  ic = select_color()
  rospy.init_node('select_color', anonymous=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
      main(sys.argv)
    except rospy.ROSInterruptException:
      pass