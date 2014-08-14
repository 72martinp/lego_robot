#!/usr/bin/python2.7

# Author: 72martinp
# Initial Date: 13.08.2014
# Last Update : 13.08.2014

# These files have been made available online through a Apache License Version 2.0
# http://www.apache.org/licenses/

# Requirements: RaspberryPI, BrickPI(tested with image based on 2014.06.05), Wifi USB module (Access Point),
# Smartphone (with Sensorstream IMU+GPS App)
#
# This code is for controlling Lego Power Functions Motors and Servo Motors over Wifi by Smartphone's Accelerometer Sensor 
# with front/rear collision detection by Lego Mindstorms NXT Ultrasonic Sensor

# This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRENTY

import socket, traceback, sys
from BrickPi import *
#from re import *

I2C_PORT_1 = PORT_1
I2C_PORT_2 = PORT_2
I2C_SPEED = 8        #define US_I2C_WAIT 7
I2C_DEVICE_INDEX = 0

LEGO_US_I2C_ADDR = 0x02

LEGO_US_CMD_REG = 0x41
LEGO_US_CMD_OFF = 0x00
LEGO_US_CMD_SS = 0x01
LEGO_US_CMD_CONT = 0x02
LEGO_US_CMD_EVNT = 0x03
LEGO_US_CMD_RST = 0x04

LEGO_US_DATA_REG = 0x42

BrickPiSetup()
# ultrasonic sensor front
BrickPi.SensorType[I2C_PORT_1] = TYPE_SENSOR_I2C_9V
BrickPi.SensorI2CSpeed[I2C_PORT_1] = I2C_SPEED
BrickPi.SensorI2CDevices[I2C_PORT_1] = 1
BrickPi.SensorSettings[I2C_PORT_1][I2C_DEVICE_INDEX] = BIT_I2C_MID | BIT_I2C_SAME
BrickPi.SensorI2CAddr[I2C_PORT_1][I2C_DEVICE_INDEX] = LEGO_US_I2C_ADDR
BrickPi.SensorI2CWrite [I2C_PORT_1][I2C_DEVICE_INDEX]    = 1
BrickPi.SensorI2CRead  [I2C_PORT_1][I2C_DEVICE_INDEX]    = 1
BrickPi.SensorI2COut   [I2C_PORT_1][I2C_DEVICE_INDEX][0] = LEGO_US_DATA_REG

# ultrasonic sensor back
BrickPi.SensorType[I2C_PORT_2] = TYPE_SENSOR_I2C_9V
BrickPi.SensorI2CSpeed[I2C_PORT_2] = I2C_SPEED
BrickPi.SensorI2CDevices[I2C_PORT_2] = 1
BrickPi.SensorSettings[I2C_PORT_2][I2C_DEVICE_INDEX] = BIT_I2C_MID | BIT_I2C_SAME
BrickPi.SensorI2CAddr[I2C_PORT_2][I2C_DEVICE_INDEX] = LEGO_US_I2C_ADDR
BrickPi.SensorI2CWrite [I2C_PORT_2][I2C_DEVICE_INDEX]    = 1
BrickPi.SensorI2CRead  [I2C_PORT_2][I2C_DEVICE_INDEX]    = 1
BrickPi.SensorI2COut   [I2C_PORT_2][I2C_DEVICE_INDEX][0] = LEGO_US_DATA_REG

# setup motors fw/bw
BrickPi.MotorEnable[PORT_A] = 1 
BrickPi.MotorEnable[PORT_B] = 1
# setup servo left/right
BrickPi.MotorEnable[PORT_D] = 1

BrickPiSetupSensors()


def move():
    #print "move"
    BrickPi.MotorSpeed[PORT_A] = SPEED  
    BrickPi.MotorSpeed[PORT_B] = SPEED  
    BrickPi.MotorSpeed[PORT_D] = STEERING 

def stop():
    #print "Stop"
    BrickPi.MotorSpeed[PORT_A] = 0  
    BrickPi.MotorSpeed[PORT_B] = 0  
    BrickPi.MotorSpeed[PORT_D] = 0

def collision_front():
    global distance_front
    distance_front = BrickPi.SensorI2CIn[I2C_PORT_1][I2C_DEVICE_INDEX][0]
    ##print distance_front

def collision_back():
    global distance_back
    distance_back = BrickPi.SensorI2CIn[I2C_PORT_2][I2C_DEVICE_INDEX][0]
    ##print distance_back


# tresholds for accerlation sensor
# TAB in Landscape view X/Y axis
t_forward = -2      # forward X=(-)
t_back = 2          # back    X=(+)
t_left = -2         # left    Y=(-)
t_right = 2         # right   Y=(+)


fw_bw= ""
left_right = ""
speed_fw_bw = ""
SPEED = ""
STEERING = ""


host = '192.168.3.1'
port = 5555
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind((host, port))

while 1:
   try:
      ## ('172882.84749, 3,   0.025, -5.853,  8.030, 4,  -0.009,  0.058,  0.018, 5,  -2.840, 43.100,-23.800', ('192.168.3.13', 57778))           

      acc, dummy = s.recvfrom(8192)
      acc_info = acc.split(",")
      ## print "acc =", acc
      ## print "dumnmy = ", dummy
      ## print "acc_info =", acc_info
     
#acc = 173348.08618, 3,  -0.271, -5.692,  8.254, 4,   0.217, -0.048,  0.048, 5,   1.410, 49.220,-31.730
#dumnmy =  ('192.168.3.13', 44972)
#acc_info = ['173348.08618', ' 3', '  -0.271', ' -5.692', '  8.254', ' 4', '   0.217', ' -0.048', '  0.048', ' 5', '   1.410', ' 49.220', '-31.730']

      # forward/backward info from accerlation sensor
      fw_bw = int(float(acc_info[2]))
      ## print "fw_bw =", fw_bw
      
      # left/right info from accerlation sensor
      left_right = int(float(acc_info[3]))
      ## print "left_right = ", left_right
      
      speed_fw_bw = abs(60*fw_bw)
	  # print speed_fw_bw
	  # set max. speed to 250
      if speed_fw_bw > 250:
          speed_fw_bw = 250
          
      steering_left_right = abs(60*left_right)
      # print steering_left_right
	  # set max. steering to 250
      if steering_left_right > 250:
          steering_left_right = 250
      
      # forward moves
      if fw_bw <= t_forward:

          # check distance forward
          collision_front()
          if distance_front < 15 :
              CurrentMove = stop
              
          # move forward left
          elif left_right <= t_left:
              SPEED = -speed_fw_bw
              STEERING = steering_left_right
              CurrentMove = move

          # move forward right
          elif left_right >= t_right:
              SPEED = -speed_fw_bw
              STEERING = -steering_left_right
              CurrentMove = move
          
          # move forward straight
          else:
              SPEED = -speed_fw_bw
              STEERING = 0                              
              CurrentMove = move
          
      # back moves            
      elif fw_bw >= t_back:
          
          # check distance back
          collision_back()
          if distance_back < 15 :
              CurrentMove = stop
          
          # move back left
          elif left_right <= t_left:
              SPEED = speed_fw_bw
              STEERING = steering_left_right
              CurrentMove = move

          # move back right
          elif left_right >= t_right:
              SPEED = speed_fw_bw
              STEERING = -steering_left_right
              CurrentMove = move
          
          # move back straight
          else:
              SPEED = speed_fw_bw
              STEERING = 0                   
              CurrentMove = move
          
      # STOP   
      else: CurrentMove = stop

      # print "SPEED= ", SPEED
      # print "STEERING= ", STEERING
     
      CurrentMove()
      BrickPiUpdateValues()       

      
   except (KeyboardInterrupt, SystemExit):
      raise
   except:
       traceback.print_exc()
