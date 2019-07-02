#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO # Control the raspberry pi header pins(python library)
import time # Get system time(python library)
from AlphaBot import AlphaBot # Movements for the robot(user defined)
import math # Mathematics operations(python library) 

Ab = AlphaBot();
Ab.stop();
# Initialization
cntl = 8;
cntr = 7;
EncR = 0.0;
EncL = 0.0;
xx = 0;
yy = 0;
theta = 0;
wheel_R = 1.25
wheel_Dia = 2*wheel_R;                 # radius of the wheels
wheel_circum = math.pi*wheel_Dia;     # circumference of the wheels

# Left encoder increment
def updateEncoderL(channel):
    global EncL;
    EncL += 1;
    #print 'EncL'


# Right encoder increment
def updateEncoderR(channel):
    global EncR;
    EncR += 1;
    #print 'EncR'


# Function to define x and y co-ordinates
def get_x_y(avg_value, r_enc, l_enc):
    global theta, xx, yy, theta_dot, wheel_circum;
    print("Initial Co-ordinates(x,y): ({0}, {1})" .format(xx, yy))
    enc_val = (r_enc+l_enc)/2
    xx = ((enc_val*wheel_circum)/40)*math.cos(theta) + xx 
    yy = ((enc_val*wheel_circum)/40)*math.sin(theta) + yy 
    theta_dot = 0
    print("Theta: %f" %theta)
    print(" Final Co-ordinates(x,y): ({0}, {1})" .format(xx, yy))
    

# Function to define the rotation value: theta
def get_theta(avg_value, r_enc, l_enc, flag):
    global theta, theta_dot
    theta_dot = 4.5*((r_enc+l_enc)/2)
    theta = theta_dot + theta
    theta = theta % 360;
    print("Theta at that instant: %f" %theta_dot)
    print("Sum of rotations - theta : %f" %theta)


def rotation(angle):
    R0 = EncR
    L0 = EncL
    flag = False
    # Anti-clockwise direction
    if(angle > 0):
        enc_val = angle*0.15 # rotation based on encoder value
        # Actual:angle*0.22   Improvised to reduce the error
        while((EncL-L0)+(EncR-R0))/2 < enc_val:
            Ab.setMotor(-35, -35);
        print('r_enc = {0} l_enc = {1}' .format((EncR-R0), (EncL-L0)));
        flag = False
        
    elif(angle < 0):# Clockwise direction
        angle = angle*-1
        enc_val = angle*0.18 # rotation based on encoder value
        # Actual angle: angle*0.22    Improvised to reduce the error
        while((EncL-L0)+(EncR-R0))/2 < enc_val:
            Ab.setMotor(35, 35);
        print('r_enc = {0} l_enc = {1}' .format((EncR-R0), (EncL-L0)));
        flag = True
    Ab.stop()
    time.sleep(0.5)
    get_theta(enc_val, (EncR-R0), (EncL-L0), flag)
    print('r_enc = {0} l_enc = {1}' .format((EncR-R0), (EncL-L0)));
    time.sleep(0.5)

def distance(distance):
    global wheel_circum;
    distance = distance*0.5;
    enc_val= (40/wheel_circum)*distance # Distance based on encoder values
    R0 = EncR
    L0 = EncL
    while(EncR-R0) < enc_val:
       if ((EncL-L0)+(EncR-R0))/2 < enc_val:
           Ab.setMotor(35, -35);
    Ab.stop()
    time.sleep(0.5)
    get_x_y(enc_val, (EncR-R0), (EncL-L0))
    print('r_enc = {0} l_enc = {1}' .format((EncR-R0), (EncL-L0)));
    time.sleep(0.5)


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False);
GPIO.setup(cntr, GPIO.IN);
GPIO.setup(cntl, GPIO.IN);
GPIO.add_event_detect(cntr, GPIO.BOTH, updateEncoderR)
GPIO.add_event_detect(cntl, GPIO.BOTH, updateEncoderL)

if __name__ == "__main__":

    Ab.stop()
    R0 = EncR;     
    L0 = EncL;

    L = 5.0; # distance between wheels
    KR = 2*math.pi/40; # Calibration factor for right wheel 
    KL = 2*math.pi/40; # calibration factor for left wheel
    #wheel_r_speed = KR*EncR; 
    #wheel_l_speed = KL*EncL;  
    #wheel_r_vel = wheel_R*EncR;    # velocity of right wheel
    #wheel_l_vel = wheel_R*EncL;    # velocity of left wheel
    Ab.stop()

    distance(12)
    rotation(45)
    distance(17)
    rotation(-90)
    distance(17)
    rotation(-45)
    distance(12)
    rotation(-45)
    distance(12)
    rotation(-45)
    distance(12)
    rotation(-90)
    distance(24)

    

