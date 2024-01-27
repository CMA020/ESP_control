#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""
 
from __future__ import print_function
 
import odrive
from odrive.enums import *
import time
import math
 
import serial
 
import keyboard
import time

import Jetson.GPIO as GPIO
import socket
content_str='K'
s = socket.socket()

s.bind(('0.0.0.0', 8090))
s.listen(5)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT, initial= GPIO.HIGH)
GPIO.setup(13, GPIO.OUT, initial= GPIO.HIGH)
GPIO.setup(16, GPIO.OUT, initial= GPIO.HIGH)
GPIO.setup(18, GPIO.OUT, initial= GPIO.HIGH)
 
coord = [0,0,0,0,0,0,0,0,0,90]
global last_command 
global last_servo 
 
last_command = ""
last_servo = ""

# # Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
my_drive0 = odrive.find_any(serial_number="207D39855841")
my_drive1 = odrive.find_any(serial_number="207039695841")
 
print("Details------------------->\n",my_drive1,"\n------------------->\n",my_drive1)
 
# # Calibrate motor and wait for it to finish
print("starting calibration...")
 
my_drive0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
my_drive0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
my_drive0.axis0.controller.config.vel_ramp_rate = 2
my_drive0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
 
my_drive0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
my_drive0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
my_drive0.axis1.controller.config.vel_ramp_rate = 2
my_drive0.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP
 
 
my_drive1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
my_drive1.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
my_drive1.axis0.controller.config.vel_ramp_rate = 2
my_drive1.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
 
my_drive1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
my_drive1.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
my_drive1.axis1.controller.config.vel_ramp_rate = 2
my_drive1.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP

 
# init serual for robotic arm
ser = serial.Serial('/dev/ttyACM1', 250000, timeout=1)
ser.reset_input_buffer()
 
 
def arm_control():
 
    global last_command
    global last_servo
 
    command = "G0 " + " X" + str(coord[4])+ " Y" +str(coord[5])+ " Z" +str(coord[6])+ " U" +str(coord[7])+ " V" +str(coord[8])+" F6000"
    servo = "M280 P1 S" + str(coord[9])
 
    if(command != last_command):
        print("Commands",command.encode()+b"\n")
        ser.write(command.encode()+b"\n")
        last_command = command
        GPIO.output(18,GPIO.LOW)
 
 
 
    if(servo != last_servo):
        print("servo",servo.encode()+b"\n")
        ser.write(servo.encode()+b"\n")
        last_servo = servo
        GPIO.output(18,GPIO.HIGH)
    # ser.write(command.encode()+b"\n")
    # time.sleep(5)
 
a = 0

state ="standby"
def chassis_move():
    global a

    if (a==0):
        my_drive0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        my_drive0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        my_drive1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        my_drive1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        a=1


 
    if(coord[0] == 1):
        my_drive0.axis1.controller.input_vel = 2
        my_drive0.axis0.controller.input_vel = 2
        my_drive1.axis1.controller.input_vel = -2 
        my_drive1.axis0.controller.input_vel = -2

    elif(coord[1] == 1):
        my_drive0.axis1.controller.input_vel = -2
        my_drive0.axis0.controller.input_vel = -2
        my_drive1.axis1.controller.input_vel = 2 
        my_drive1.axis0.controller.input_vel = 2

    elif(coord[2] == 1):
        my_drive0.axis1.controller.input_vel = 2
        my_drive0.axis0.controller.input_vel = 2
        my_drive1.axis1.controller.input_vel = -0.5
        my_drive1.axis0.controller.input_vel = -0.5

    elif(coord[3] == 1):
        my_drive0.axis1.controller.input_vel = 0.5
        my_drive0.axis0.controller.input_vel = 0.5
        my_drive1.axis1.controller.input_vel = -2
        my_drive1.axis0.controller.input_vel = -2


    if (coord[0]==0 and coord[1]==0 and coord[2]==0 and coord[3]==0 ):
        my_drive0.axis1.controller.input_vel = 0
        my_drive0.axis0.controller.input_vel = 0
        my_drive1.axis1.controller.input_vel = 0 
        my_drive1.axis0.controller.input_vel = 0        

        my_drive0.axis0.requested_state = AXIS_STATE_IDLE
        my_drive0.axis1.requested_state = AXIS_STATE_IDLE
        my_drive1.axis0.requested_state = AXIS_STATE_IDLE
        my_drive1.axis1.requested_state = AXIS_STATE_IDLE
        a=0

 
 
# Constants
INPUT_INTERVAL = 0.01
COMMAND_INTERVAL = 1
last_input_time = time.time()
last_command_time = time.time()


def lights_control():
    if(coord[0]==coord[1]==coord[2]==coord[3]):
        GPIO.output(13, GPIO.LOW)
        GPIO.output(11, GPIO.HIGH)
    else:
        GPIO.output(11, GPIO.LOW)
        GPIO.output(13, GPIO.HIGH)




if __name__ == '__main__':
 
    while True:


        try:
            client, addr = s.accept()
            while True:
                content = client.recv(32)
                content_str = content.decode('utf-8')
                # print(content)
                # time.sleep(0.5)
                if len(content_str) == 0:
                    print("nothing")

                else:

                    print(content_str)
                    break

            print("guwda")
            print(content_str)
            # Check if it's time to take input
            if time.time() - last_command_time >= INPUT_INTERVAL:
                # Take input from the keyboard
 
                if (content_str=="q"):  # Quit if 'q' is pressed
                    print("Closing Robot Control\n")
                    break
 
                # robot chassis control
                if (content_str=="W"):
                    coord[0] = 1
                    coord[1] = 0
                    coord[2] = 0
                    coord[3] = 0
                    # Code to control the forward motion of the robot
 
                elif (content_str=="S"):
                    coord[1] = 1
                    coord[0] = 0
                    coord[2] = 0
                    coord[3] = 0
                    # Code to control the backward motion of the robot
 
                elif (content_str=="A"):
                    coord[2] = 1
                    coord[0] = 0
                    coord[1] = 0
                    coord[3] = 0
                    # Code to control the backward motion of the robot
 
                elif (content_str=="D"):
                    coord[3] = 1
                    coord[0] = 0
                    coord[1] = 0
                    coord[2] = 0
                    # Code to control the backward motion of the robot
                elif (content_str=="Z"):
                    coord[0] = 0
                    coord[1] = 0
                    coord[2] = 0
                    coord[3] = 0
                    coord[4] = 0
                    coord[5] = 0
                    coord[6] = 0
                    coord[7] = 0
                    coord[8] = 0
                    coord[9] = 0
                    

 
                # arm link 1 control
                if (content_str == "T"):
                    coord[4] = coord[4] + 1

                elif (content_str == "G"):
                    coord[4] = coord[4] - 1
                    # Code to control the sideways motion to the right of the robot
 
                elif keyboard.is_pressed('v'):
                    coord[4] = 0
                    # Code to control the forward motion of the robot
 
                # arm link2
                if (content_str == "Y"):
                    coord[5] = coord[5] + 1
                    # Code to control the sideways motion to the left of the robot
 
                elif (content_str == "H"):
                    coord[5] = coord[5] - 1
                    # Code to control the sideways motion to the right of the robot
 
                elif keyboard.is_pressed('b'):
                    coord[5] = 0
                    # Code to control the forward motion of the robot
 
                # link3
                if (content_str == "U"):
                    coord[6] = coord[6] + 1
                    # Code to control the sideways motion to the left of the robot
 
                elif (content_str == "J"):
                    coord[6] = coord[6] - 1
                    # Code to control the sideways motion to the right of the robot
 
                elif keyboard.is_pressed('n'):
                    coord[6] = 0
                    # Code to control the forward motion of the robot
 
                # link 4
                if (content_str == "I"):
                    coord[7] = coord[7] + 1
                    # Code to control the sideways motion to the left of the robot
 
                elif (content_str == "K"):
                    coord[7] = coord[7] - 1
                    # Code to control the sideways motion to the right of the robot
 
                elif keyboard.is_pressed('m'):
                    coord[7] = 0
                    # Code to control the forward motion of the robot
 
                # link 5
                if (content_str == "O"):
                    coord[8] = coord[8] + 1
                    # Code to control the sideways motion to the left of the robot
 
                elif (content_str == "L"):
                    coord[8] = coord[8] - 1
                    # Code to control the sideways motion to the right of the robot
 
                elif keyboard.is_pressed(','):
                    coord[8] = 0
                    # Code to control the forward motion of the robot
 
                # servo
                if (content_str == "P"):
                    coord[9] = coord[9] + 1
                    # Code to control the sideways motion to the left of the robot
 
                elif keyboard.is_pressed(';'):
                    coord[9] = coord[9] - 1
                    # Code to control the sideways motion to the right of the robot
 
                elif keyboard.is_pressed('.'):
                    coord[9] = 0
                    # Code to control the forward motion of the robot
 
 
                last_input_time = time.time()
 
            # Check if it's time to send a command
            if time.time() - last_command_time >= COMMAND_INTERVAL:
                # Other operations or code here
                arm_control()
                chassis_move()
                lights_control()
 
                last_command_time = time.time()
            print("Closing connection")
            client.close()
            # Delay to control the loop frequency
            time.sleep(0.01)
 
        except KeyboardInterrupt:
                    break
