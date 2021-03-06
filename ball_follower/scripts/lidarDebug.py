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
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError



class lidarDebug:

  def __init__(self):
    self.angle_sub = rospy.Subscriber("/control/angle/setpoint", Point, self.angle_cb)        
    self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_cb)

    self.radians = 0
    self.degrees = 0

    self.degrees_per_frame = 50
    self.radians_per_frame = self.degrees_per_frame*3.14159/180

    self.intensities = []

    self.window = 4

    self.data_recieved = False
    return




  def angle_cb(self,data):
    self.radians = (0.5-data.x)*self.radians_per_frame  
    self.degrees = (0.5-data.x)*self.degrees_per_frame  
    return


  def lidar_cb(self,data):

    # Lidar
    # angle_min, angle_max
    # time_increment
    # range_min, range_max
    # ranges, intensities

    self.ranges = data.ranges
    self.intensities = data.intensities

    self.data_recieved = True
    return

  def plot(self):
    # Crop Lidar to Image Frame
    half_width = int(self.degrees_per_frame/2)

    # Intensities
    cropped_left = list(self.intensities[0:half_width])[::-1]
    cropped_right = list(self.intensities[-half_width:-1])[::-1]
    cropped = cropped_left+cropped_right
    intensities = cropped

    cropped_left = list(self.ranges[0:half_width])[::-1]
    cropped_right = list(self.ranges[-half_width:-1])[::-1]
    cropped = cropped_left+cropped_right
    ranges = cropped

    # Angle Overlay
    angle_i = min(max(0,int(half_width-self.degrees)),len(cropped)-1)

    # Find Max Intensity Within 6 Degrees
    limited_i = intensities[max(0,(angle_i-self.window)):min((angle_i+self.window),len(intensities)-1)]
    limited_r = ranges[max(0,(angle_i-self.window)):min((angle_i+self.window),len(ranges)-1)]
    i = limited_i.index(max(limited_i))

    r_guess = limited_r[i]
    print r_guess



    overlay = [0]*len(cropped)
    overlay[angle_i] = 1


    plt.clf()

    max_intensity = max(intensities)
    norm_i = [i/max_intensity for i in intensities]

    max_range = max(ranges)
    norm_r = [r/max_range for r in ranges]

    out = norm_i
    out2 = norm_r

    #plt.plot(X,Y)
    plt.plot(range(len(out)),out)
    plt.plot(range(len(out2)),out2)
    plt.plot(range(len(overlay)),overlay)

    plt.draw()
    plt.pause(0.00000000001)

    return


def main(args):
  ic = lidarDebug()
  rospy.init_node('lidarDebug', anonymous=True)

  plt.ion()
  plt.show()

  try:
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      
      if ic.data_recieved:
        ic.plot()
      rate.sleep()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
      main(sys.argv)
    except rospy.ROSInterruptException:
      pass