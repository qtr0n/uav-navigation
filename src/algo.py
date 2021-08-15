#! /usr/bin/env python
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from std_msgs.msg import Float64
import rospy
import time
import argparse 
from std_msgs.msg import Int64

position = "straight"
turn_right = 0
turn_left = 0
check_turn = 0
right = 0
left = 0
forward = 1
right_wall = 0
left_wall = 0 
rotating = 0
start = 0
last_wall = "none"
turn = 0
height_up = 0
height_down = 0
count = 0
turned = 0
height_count = 10
centre = 0
this_height = 0
this_time = 1
both = 0
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)
#921600 is the baudrate that you have set in the mission plannar or qgc
def isnan(num):
    return num != num
# Function to arm and then takeoff to a user specified altitude
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

arm_and_takeoff(1.5)


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




def call(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global flag
    
    flag = data.data
    
    # print(data.data)
    # print(flag)


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global position
    global turn_right
    global turn_left
    global check_turn
    global right 
    global left 
    global forward 
    global right_wall
    global left_wall  
    global rotating
    global start
    global last_wall
    global turn
    global height_up
    global height_down
    global count
    global turned
    global height_count
    global centre
    global this_height
    global this_time
    global both
    
    if(flag==0):
        print("Marker ID: none, looking for marker")
        if(rotating > 37):
            if((data.data < 1.5 and last_wall != "none" and turned == 1) or (turn == 1) or (centre == 1) or (height_up == 1) or (height_down == 1)):
                if(this_time == 1):
                    both = 0
                    if(height_count == 60):
                        this_height = 0
                    elif(height_count == 10):
                        this_height = 1
                    else:
                        both = 1
                    this_time = 0

                if(height_up == 1):
                    if(data.data < 2 and data.data != 0.0 and height_count < 60 and (this_height == 1 or both == 1) ):
                        send_ned_velocity(0,0,-0.4,1)
                        height_count = height_count + 1
                    elif(data.data > 2 or data.data == 0.0):
                        send_ned_velocity(0,0,0,1)
                        last_wall = "none"
                        this_time = 1
                        turned = 0
                        height_up = 0
                    else:
                        send_ned_velocity(0,0,0,1)
                        rotating = 32
                        height_up = 0
                        # count = 0
                        centre = 1
                elif(centre == 1):                 
                    if(last_wall == "right"):
                        if(data.data < 3 and data.data != 0.0 and count < 100):
                            count = count + 1
                            if(count < 70):
                                if(position == "straight"):
                                    send_ned_velocity(0.3,0,0,1)
                                elif(position == "left"):
                                    send_ned_velocity(0,-0.3,0,1)
                                elif(position == "right"):
                                    send_ned_velocity(0,0.3,0,1)
                                else:
                                    send_ned_velocity(-0.3,0,0,1)
                        elif(data.data > 3 or data.data == 0):
                            centre = 0
                            last_wall = "none"
                            turned = 0
                            this_time = 1
                            rotating = 30
                            count = 0
                        else:
                            turn = 1
                            centre = 0
                            count = 0
                            rotating = 0
                    elif(last_wall == "left"):
                        if(data.data < 3 and data.data != 0.0 and count < 100):
                            count = count + 1
                            if(count < 60):
                                if(position == "straight"):
                                    send_ned_velocity(-0.3,0,0,1)
                                elif(position == "left"):
                                    send_ned_velocity(0,0.3,0,1)
                                elif(position == "right"):
                                    send_ned_velocity(0,-0.3,0,1)
                                else:
                                    send_ned_velocity(0.3,0,0,1)
                        elif(data.data > 3  or data.data == 0):
                            centre = 0
                            last_wall = "none"
                            turned = 0
                            this_time = 1
                            rotating = 32
                            count = 0 
                        else:
                            turn = 1
                            centre = 0
                            count = 0
                            rotating = 0
                elif(turn == 1):
                    if(check_turn == 0):
                        condition_yaw(270,1)
                        rotating = 0
                        check_turn = 1
                        rotating = 0
                    elif(check_turn == 1):
                        turn_left = data.data
                        check_turn = 2
                        rotating = 20
                    elif(check_turn == 2):
                        condition_yaw(180,1)
                        rotating = 0
                        check_turn = 3
                        rotating = 0
                    elif(check_turn == 3):
                        turn_right = data.data
                        check_turn = 4
                        rotating = 20
                    else:
                        if((turn_left < turn_right) or turn_left == 0.0):
                            condition_yaw(180,1)
                            rotating = 20
                            if(position == "straight"):
                                position = "left"
                            elif(position == "left"):
                                position = "back"
                            elif(position == "right"):
                                position == "straight"
                            else:
                                position = "right"
                        else:
                            if(position == "straight"):
                                position = "right"
                            elif(position == "left"):
                                position = "straight"
                            elif(position == "right"):
                                position == "back"
                            else:
                                position = "left"
                        check_turn = 0
                        turn = 0
                        rotating = 30
                        turned = 0
                        last_wall = "none"
                        turn_right = 0
                        turn_left = 0
                        right = 0
                        left = 0
                        this_time = 1
                        forward = 1
                        right_wall = 0
                        left_wall = 0 
                        rotating = 0
                        start = 0
                else:
                    height_down = 1
                    if(data.data < 2 and data != 0.0 and height_count > 10 and (this_height == 0 or both == 1)):
                        send_ned_velocity(0,0,0.4,1)
                        height_count = height_count - 1
                    elif(data.data > 2 or data == 0.0):
                        send_ned_velocity(0,0,0,1)
                        rotating = 30
                        last_wall = "none"
                        height_down = 0
                        this_time = 1
                        turned = 0
                    else:
                        send_ned_velocity(0,0,0,1)
                        rotating = 30
                        height_up = 1
                        height_down = 0

            elif(data.data > 2 or data.data == 0):
                if(forward == 1): 
                    last_wall = "none"
                    turned = 0
                    if(start < 10):
                        if(position == "straight"):
                            send_ned_velocity(0,-0.1,0,1)
                        elif(position == "left"):
                            send_ned_velocity(-0.1,0,0,1)
                        elif(position == "right"):
                            send_ned_velocity(0.1,0,0,1)
                        else:
                            send_ned_velocity(0,0.1,0,1)
                        start = start + 1
                    else:
                        if(position == "straight"):
                            send_ned_velocity(0,-0.6,0,1)
                        elif(position == "left"):
                            send_ned_velocity(-0.6,0,0,1)
                        elif(position == "right"):
                            send_ned_velocity(0.6,0,0,1)
                        else:
                            send_ned_velocity(0,0.6,0,1)
                elif(right == 1): 
                    if(start < 10):
                        if(position == "straight"):
                            send_ned_velocity(0.1,0,0,1)
                        elif(position == "left"):
                            send_ned_velocity(0,-0.1,0,1)
                        elif(position == "right"):
                            send_ned_velocity(0,0.1,0,1)
                        else:
                            send_ned_velocity(-0.1,0,0,1)
                        start = start + 1
                    else:
                        if(position == "straight"):
                            send_ned_velocity(0.6,0,0,1)
                        elif(position == "left"):
                            send_ned_velocity(0,-0.6,0,1)
                        elif(position == "right"):
                            send_ned_velocity(0,0.6,0,1)
                        else:
                            send_ned_velocity(-0.6,0,0,1)
                elif(left == 1): 
                    if(start < 10):
                        if(position == "straight"):
                            send_ned_velocity(-0.1,0,0,1)
                        elif(position == "left"):
                            send_ned_velocity(0,0.1,0,1)
                        elif(position == "right"):
                            send_ned_velocity(0,-0.1,0,1)
                        else:
                            send_ned_velocity(0.1,0,0,1)
                        start = start + 1
                    else:
                        if(position == "straight"):
                            send_ned_velocity(-0.6,0,0,1)
                        elif(position == "left"):
                            send_ned_velocity(0,0.6,0,1)
                        elif(position == "right"):
                            send_ned_velocity(0,-0.6,0,1)
                        else:
                            send_ned_velocity(0.6,0,0,1)
            elif(data.data <= 2.0 and data.data >= 1.5):
                turned = 0
                if(forward == 1):
                    if(position == "straight"):
                        send_ned_velocity(0,-0.2,0,1)
                    elif(position == "left"):
                        send_ned_velocity(-0.2,0,0,1)
                    elif(position == "right"):
                        send_ned_velocity(0.2,0,0,1)
                    else:
                        send_ned_velocity(0,0.2,0,1)
                elif(right == 1):
                    print("right_move_2") 
                    if(position == "straight"):
                        send_ned_velocity(0.2,0,0,1)
                    elif(position == "left"):
                        send_ned_velocity(0,-0.2,0,1)
                    elif(position == "right"):
                        send_ned_velocity(0,0.2,0,1)
                    else:
                        send_ned_velocity(-0.2,0,0,1)
                elif(left == 1): 
                    if(position == "straight"):
                        send_ned_velocity(-0.2,0,0,1)
                    elif(position == "left"):
                        send_ned_velocity(0,0.2,0,1)
                    elif(position == "right"):
                        send_ned_velocity(0,-0.2,0,1)
                    else:
                        send_ned_velocity(0.2,0,0,1)
            else:
                send_ned_velocity(0,0,0,1)
                if(forward == 1):
                    if(left_wall == 1):
                        condition_yaw(90,1)
                        rotating = 32
                        rotating = 0
                        left_wall = 0
                        right = 1
                        forward = 0
                        start = 0
                        last_wall = "left"
                    elif(right_wall == 1):
                        condition_yaw(270,1)
                        rotating = 32
                        rotating = 0
                        right_wall = 0
                        left = 1
                        forward = 0
                        start = 0
                        last_wall = "right"
                    else:
                        condition_yaw(270,1)
                        rotating = 32
                        rotating = 0
                        right_wall = 0
                        left = 1
                        forward = 0
                        start = 0
                elif(right == 1):
                    if(last_wall != "none"):
                        turned = 1
                    condition_yaw(270,1)
                    rotating = 32
                    rotating = 0
                    right_wall = 1
                    right = 0
                    forward = 1
                    start = 0
                elif(left == 1):
                    if(last_wall != "none"):
                        turned = 1
                    condition_yaw(90,1)
                    rotating = 32
                    rotating = 0
                    left_wall = 1
                    left = 0
                    forward = 1
                    start = 0
        
        rotating = rotating + 1
        
    elif(flag!=0):
        send_ned_velocity(0,0,0,1)
        if(flag==2):
            send_ned_velocity(0.2,-0.2,0,1)
        elif(flag==3):
            send_ned_velocity(0.3,0,0,1)                    
        elif(flag==4):
            send_ned_velocity(0,-0.3,0,1)   
        elif(flag==5):
            send_ned_velocity(-0.2,-0.2,0,1) 
        elif(flag==6):
            send_ned_velocity(-0.3,0,0,1)
        elif(flag==7):
            send_ned_velocity(0,-0.3,0,1)
        elif(flag==8):
            vehicle.mode=VehicleMode("LAND")
            print("Marker ID : 0, Landed")

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
        rospy.init_node('listener', anonymous=True)

      
        rospy.Subscriber("/Aruco/message", Int64, call) 
        rospy.Subscriber("chatter",Float64, callback)    # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    listener()
vehicle.close()    