#! /usr/bin/env python
import rospy
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from sensor_msgs.msg import Image
from std_msgs.msg import Int64
from cv_bridge import CvBridge
import cv2
import os
import time
import numpy as np
import cv2.aruco as aruco
import vector as vector
from numpy import double
import argparse 

flag = 0
temp = 0
bo=0
random=0

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
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
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(0.2)

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

class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)
    
        # Publishers
        self.pub = rospy.Publisher('detected_image', Image,queue_size=10)
        self.pubb = rospy.Publisher('/Aruco/message', Int64, queue_size=10)
        # Subscribers
        rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

        self.image_new = None

    def callback(self, msg):
        rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)
        self.track(cameraMatrix,distMatrix)
        
            
            #self.loop_rate.sleep()
 
      

    def start(self):
        rospy.loginfo("Timing images")
        #rospy.spin()
        while not rospy.is_shutdown():
            
            #br = CvBridge()
            if self.image_new is not None:
                self.pub.publish(self.br.cv2_to_imgmsg(self.image))
            self.loop_rate.sleep()



    def track(self,matrix_coefficients,distortion_coefficients):
           
            frame = self.image
            # operations on the frame come here
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
            aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
            parameters = aruco.DetectorParameters_create()  # Marker detection parameters
            # lists of ids and the corners beloning to each id
            corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            if np.all(ids is not None):  # If there are markers found by detector
                for i in range(0, len(ids)):  # Iterate in markers
                    # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                    rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,distortion_coefficients)
                    (rvec - tvec).any()  # get rid of that nasty numpy value array error
                    aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                    rospy.loginfo(i)
                    print(tvec)
                    x=tvec[0][0][0]
                    print(x)
                    y=tvec[0][0][1]
                    print(y)
                    z=tvec[0][0][2]
                    print(z)
                    
                    global flag
                    global temp
                    if(tvec.size>0):
                        flag = 1
                    print(flag)
                    if(flag==0):
                        self.pubb.publish(Int64(flag))                   
        
                    elif(flag!=0):
                        global bo
                        global random
                        flag=1
                        self.pubb.publish(Int64(flag))
                        if(random>20 and random<40):
                            if(bo==0):
                                y=(270-vehicle.heading+360)%360
                                condition_yaw(y,1)
                                bo=1
                            send_ned_velocity(0,0,0,1)
                        elif(random>40):
                            if(x>0.005 and y<-0.005):
                                flag=2
                                self.pubb.publish(Int64(flag))
                                
                            elif(x>0.005 and y>-0.005):
                                flag=3
                                self.pubb.publish(Int64(flag))
                                    
                            elif(x<0.005 and y<-0.005):
                                flag=4
                                self.pubb.publish(Int64(flag))
                            elif(x<-0.005 and y<-0.005):
                                flag=5
                                self.pubb.publish(Int64(flag))
                                                   
                            elif(x<-0.005 and y>-0.005):
                                flag=6
                                self.pubb.publish(Int64(flag))
                                    
                            elif(x>-0.005 and y<-0.005):
                                flag=7
                                self.pubb.publish(Int64(flag))
                            else:
                                flag=8
                                self.pubb.publish(Int64(flag))
                        random = random +1
              
                    frame = cv2.putText(frame, np.array_str(ids), (50, 50), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 0, 0), 1, cv2.LINE_AA)
                    aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)
            self.image_new=frame
            if self.image_new is not None:
                  self.pub.publish(self.br.cv2_to_imgmsg(self.image_new))
                  rospy.loginfo('publishing image')
                      # Draw Axis
                    #frame = cv2.putText(frame, np.array_str(tvec),(30,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)


def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]


cameraMatrix, distMatrix = load_coefficients("test.txt")
print(cameraMatrix)
print(distMatrix)

# cmatrix = np.arange(9).reshape(3,3)
# dmatrix = np.arange(5).reshape(1,5)
# dmatrix = np.array([0,0,0,0,0])
# cmatrix = np.array([

#        [56.90422362262635, 0, 300.0881775819985],
#        [0, 56.91071703888348, 234.2122896267271],
#        [0, 0, 1]
#     ])

# print(cmatrix)
# print(dmatrix)


          

if __name__ == '__main__':
    rospy.init_node("imagetimer111", anonymous=True)
    # Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
    # just like before we specify an enum flag, but this time it is 
    my_node = Nodo()
    my_node.start()