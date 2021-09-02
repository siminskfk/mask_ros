#!/usr/bin/env python3

import Jetson.GPIO as GPIO
import rospy
from std_msgs.msg import Int32
import roslib
from adafruit_servokit import ServoKit

# -------------------- Code for Motor -------------------- #

# Pins setting for DC Motor
PWM = [32, 33]

def gpio_setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PWM, GPIO.OUT, initial=GPIO.LOW)
    motor = GPIO.PWM(PWM, 50)
    servo = ServoKit(channels=16)

def callback(msg):
    print(msg.data)
    if msg.data == 1:
        motor.start(20)
        motor.ChangeDutyCycle(12)
        print("go")

    elif msg.data == 2:
        motor.ChangeDutyCycle(0)
        print("stop")

    elif msg.data >= 45 and msg.data <= 135:
        kit.servo[0].angle = msg.data
        print("Curve angle :", msg.data)

# -------------------- Main -------------------- # 

if __name__ == "__main__":
    gpio_setup()
    rospy.init_node('motor')
    sub = rospy.Subscriber('mask', Int32, callback)

    rospy.spin()

