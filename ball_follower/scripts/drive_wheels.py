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


class drive_wheels:

  def __init__(self):


    #self.image_pub = rospy.Publisher("/image/compressed", CompressedImage, queue_size=1)    
    #self.point_pub = rospy.Publisher("/control/set_point", Point, queue_size=1)

    self.control_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    self.setpoint_sub = rospy.Subscriber("/control/setpoint", Point, self.setpoint_cb)
  
    self.x_pct = 0.5
    self.y_pct = 0.5
    self.fill_pct = 0



    return

  def vels(self,target_linear_vel, target_angular_vel):
      return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)



  def setpoint_cb(self,data):

    self.x_pct = data.x
    self.y_pct = data.y
    self.fill_pct = data.z

    print "IN CONTROL NODE",data.x, data.y, data.z


    status = 0
    target_linear_vel = 0
    target_angular_vel = 0
    control_linear_vel = 0
    control_angular_vel = 0

    #if self.fill_pct >= 0.2 :
    #  target_linear_vel = target_linear_vel + 0.01
    #  status = status + 1
    #  print self.vels(target_linear_vel,target_angular_vel)
    #elif self.fill_pct <= 0.5 :
    #  target_linear_vel = target_linear_vel - 0.01
    #  status = status + 1
    #  print self.vels(target_linear_vel,target_angular_vel)
    if self.x_pct < 0.4 :
      target_angular_vel = target_angular_vel + 0.1
      status = status + 1
      print self.vels(target_linear_vel,target_angular_vel)
    elif self.x_pct > 0.6 :
      target_angular_vel = target_angular_vel - 0.1
      status = status + 1
      print self.vels(target_linear_vel,target_angular_vel)
    elif self.fill_pct <= 0.01 :
      target_linear_vel   = 0
      control_linear_vel  = 0
      target_angular_vel  = 0
      control_angular_vel = 0
      print vels(0, 0)
    elif status == 14 :
      print msg
      status = 0


    if target_linear_vel > control_linear_vel:
      control_linear_vel = min( target_linear_vel, control_linear_vel + (0.01/4.0) )
    else:
      control_linear_vel = target_linear_vel

    if target_angular_vel > control_angular_vel:
      control_angular_vel = min( target_angular_vel, control_angular_vel + (0.1/4.0) )
    else:
      control_angular_vel = target_angular_vel

    twist = Twist()
    twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_angular_vel
    self.control_pub.publish(twist)



    return





def main(args):
  ic = drive_wheels()
  rospy.init_node('drive_wheels', anonymous=True)

  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    rate.sleep()




if __name__ == '__main__':
    try:
      main(sys.argv)
    except rospy.ROSInterruptException:
      pass