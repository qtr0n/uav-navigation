#! /usr/bin/env python
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import argparse  
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import Float64
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()
k = 0
def talker_callback(data):
    pub = rospy.Publisher('chatter', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
def arm_and_takeoff(aTargetAltitude):

        print ("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not vehicle.is_armable:
          print (" Waiting for vehicle to initialise...")
          time.sleep(1)
              
        print ("Arming motors")
        # Copter should arm in GUIDED mode
        vehicle.mode    = VehicleMode("GUIDED")
        vehicle.armed   = True

        while not vehicle.armed:
          print (" Waiting for arming...")
          time.sleep(1)

        print ("Taking off!")
        vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Check that vehicle has reached takeoff altitude
        while True:
          print (" Altitude: ", vehicle.location.global_relative_frame.alt)
          #Break and return from function just below target altitude.        
          if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
            print ("Reached target altitude")
            break
          time.sleep(1)



class WallFollower(object):
    def __init__(self):
        self.image_sub=rospy.Subscriber("/depth_camera/depth/image_raw",Image,self.camera_callback)
        self.bridge_object=CvBridge()
        self.sum_pub = rospy.Publisher('chatter', Float64)

    def camera_callback(self,data):
        try:
            cv_image=self.bridge_object.imgmsg_to_cv2(data, desired_encoding="32FC1")
        except CvBridgeError as e:
            print(e)
        sum1 = 0
        sum2 = 0
        sum3 = 0
        global k
        x = 1
        if(k%3 == 0): 
          x=1
          for i in range(210,230):
            for j in range(130,150):
              if(math.isnan(cv_image[i][j])==False):
                sum1 += cv_image[i][j]
                x = x+1
          sum1 = sum1/x
          x = 1
          for i in range(210,230):
              for j in range(250,270):
                if(math.isnan(cv_image[i][j])==False):
                  sum2 += cv_image[i][j]
                  x = x+1
          sum2 = sum2/x
          x = 1
          for i in range(210,230):
            for j in range(380,400):
              if(math.isnan(cv_image[i][j])==False):
                sum3 += cv_image[i][j]
                x = x+1
          sum3 = sum3/x
          smallest = 0
          if sum1 < sum2 and sum1 < sum3 :
              if(sum1 == 0):
                smallest = min(sum2,sum3)
              else:
                smallest = sum1
          elif sum2 < sum1 and sum2 < sum3 :
              if(sum2 == 0):
                smallest = min(sum1,sum3)
              else:
                smallest = sum2
          else:
              if(sum3 == 0):
                if(sum2 != 0):
                    smallest = min(sum2,sum3)
                else:
                    smallest = sum1
              else:
                smallest = sum3
          try:
              self.sum_pub.publish(Float64(smallest))
          except CvBridgeError as e:
              print(e) 
        k = k + 1   
      

    
    
def send_ned_velocity(velocity_x, velocity_y, velocity_z,duration):

  msg = vehicle.message_factory.set_position_target_local_ned_encode(
      0,       # time_boot_ms (not used)
      0, 0,    # target system, target component
      mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
      0b0000111111000111, # type_mask (only speeds enabled)
      0, 0, 0, # x, y, z positions (not used)
      velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
      0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
      0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
  # send command to vehicle on 1 Hz cycle
  while True:
      vehicle.send_mavlink(msg)

      if(flag):
        break

      time.sleep(1)  


def condition_yaw(heading, relative=False):
  if relative:
      is_relative=1 #yaw relative to direction of travel
  else:
      is_relative=0 #yaw is an absolute angle
  # create the CONDITION_YAW command using command_long_encode()
  msg = vehicle.message_factory.command_long_encode(
      0, 0,    # target system, target component
      mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
      0, #confirmation
      heading,    # param 1, yaw in degrees
      0,          # param 2, yaw speed deg/s
      1,          # param 3, direction -1 ccw, 1 cw
      is_relative, # param 4, relative offset 1, absolute angle 0
      0, 0, 0)    # param 5 ~ 7 not used
  # send command to vehicle
  vehicle.send_mavlink(msg)
          

# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)
#921600 is the baudrate that you have set in the mission plannar or qgc


wall_follower_object=WallFollower()
print(wall_follower_object)
rospy.init_node('line_following_node',anonymous=True)
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
  
vehicle.close()    
