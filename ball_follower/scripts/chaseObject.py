#!/usr/bin/env python
import roslib
roslib.load_manifest('ball_follower')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point


class chaseObject:

  def __init__(self):


    #self.image_pub = rospy.Publisher("/image/compressed", CompressedImage, queue_size=1)    
    #self.point_pub = rospy.Publisher("/control/set_point", Point, queue_size=1)

    self.control_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    self.image_setpoint_sub = rospy.Subscriber("/control/angle/setpoint", Point, self.angle_setpoint_cb)
    self.lidar_setpoint_sub = rospy.Subscriber("/control/range/setpoint", Point, self.range_setpoint_cb)
  
    self.x_pct = 0.5
    self.y_pct = 0.5
    self.fill_pct = 0

    self.range = 0.5

    return


  def angle_setpoint_cb(self,data):
    self.x_pct = data.x
    self.y_pct = data.y
    self.fill_pct = data.z
    return

  def range_setpoint_cb(self,data):
    self.range = data.x
    return


  def publish_control(self):
    
    x_desired = 0.5
    x_current = self.x_pct

    dist_desired = 0.5 # meters
    dist_current = self.range
    

    # PID Constants
    p_angular = 1;
    p_linear = 1;

    # Default State (Do Nothing)
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0




    if self.fill_pct > 0.01:
      # Angular Velocity Control
      twist.angular.z = p_angular*(x_desired-x_current)      

      # Linear Velocity Control
      twist.linear.x = p_linear*(dist_current-dist_desired)





    self.control_pub.publish(twist)
    return 





def main(args):
  ic = chaseObject()
  rospy.init_node('chaseObject', anonymous=True)

  rate = rospy.Rate(5)
  while not rospy.is_shutdown():
    ic.publish_control()
    #print "CONTROL [",5,"Hz] Image: ",ic.x_pct,ic.y_pct,"Lidar:"
    rate.sleep()




if __name__ == '__main__':
    try:
      main(sys.argv)
    except rospy.ROSInterruptException:
      pass