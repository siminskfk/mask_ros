#!/usr/bin/env python3

import Jetson.GPIO as GPIO
import rospy
from std_msgs.msg import Int32
import roslib

# -------------------- Initialize for Motor -------------------- #

# Pins setting for DC Motor
ENA = [19, 29]
MOTOR = [21, 31]
DIR = [23, 33]

run = 0

# -------------------- Code for Motor -------------------- # 

def gpio_setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(DIR, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(MOTOR, GPIO.OUT, initial=GPIO.LOW)

def forward():
    print("Forward")
    GPIO.output(DIR, GPIO.LOW)
    GPIO.output(MOTOR, GPIO.HIGH)

def backward():
    print("Backward")
    GPIO.output(DIR, GPIO.HIGH)
    GPIO.output(MOTOR, GPIO.HIGH)

def stop():
    print("Stop")
    GPIO.output(MOTOR, GPIO.LOW)

def callback(msg):
    print(msg.data)
    if msg.data > 0:
        run = 0
        print("stop")

    else:
        run = 1
        print("go")

# -------------------- Main -------------------- # 

if __name__ == "__main__":
    gpio_setup()
    rospy.init_node('motor')
    sub = rospy.Subscriber('mask', Int32, callback)

    if run == 0:
        stop()
    elif run == 1:
        forward()

    rospy.spin()

