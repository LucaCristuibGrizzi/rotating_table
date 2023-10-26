#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool, Int16
import RPi.GPIO as GPIO
import time
import threading, sys

pub = rospy.Publisher('rs_rotating_table', Bool, queue_size=1)

in1 = 17
in2 = 18
in3 = 27
in4 = 22

speedLock = threading.Lock()
componentErrorLock = threading.Lock()

componentError = False
nComponentError = 0
counterComponentError = 0

# careful lowering this, at some point you run into the mechanical limitation of how quick your motor can move
step_sleep = 0.001

step_count = 11605 # 5.625*(1/64) per step, 4096 steps is 360°(85/15= 5.6667 -> 5.6667/2 = 2.8333 -> 2.8333*4096 = ~ 11605)

direction = False # True for clockwise, False for counter-clockwise

# defining stepper motor sequence 
# (found in documentation http://www.4tronix.co.uk/arduino/Stepper-Motors.php)
step_sequence = [[1,0,0,1],
                 [1,0,0,0],
                 [1,1,0,0],
                 [0,1,0,0],
                 [0,1,1,0],
                 [0,0,1,0],
                 [0,0,1,1],
                 [0,0,0,1]]

# setting up
GPIO.setmode( GPIO.BCM )
GPIO.setup( in1, GPIO.OUT )
GPIO.setup( in2, GPIO.OUT )
GPIO.setup( in3, GPIO.OUT )
GPIO.setup( in4, GPIO.OUT )

# initializing
GPIO.output( in1, GPIO.LOW )
GPIO.output( in2, GPIO.LOW )
GPIO.output( in3, GPIO.LOW )
GPIO.output( in4, GPIO.LOW )

motor_pins = [in1,in2,in3,in4]

def getStepSleep():
    with speedLock :
        return step_sleep
    
def setStepSleep(sleep):
    with speedLock :
        global step_sleep
        step_sleep = sleep

def getCounterComponentError():
    with componentErrorLock:
        return counterComponentError
    
def addCounterComponentError():
    with componentErrorLock:
        global counterComponentError
        counterComponentError =+ 1


def cleanup():
    lowOutputPin()
    GPIO.cleanup()

def lowOutputPin():
    GPIO.output( in1, GPIO.LOW )
    GPIO.output( in2, GPIO.LOW )
    GPIO.output( in3, GPIO.LOW )
    GPIO.output( in4, GPIO.LOW )

def rotate(data):
    if False:
        speed = 60
        pub.publish(str(speed))
        return

    motor_step_counter = 0
    sleep = getStepSleep()
    for i in range(step_count):
        for pin in range(0, len(motor_pins)):
            GPIO.output( motor_pins[pin], step_sequence[motor_step_counter][pin] )
        if direction==True:
            motor_step_counter = (motor_step_counter - 1) % 8
        elif direction==False:
            motor_step_counter = (motor_step_counter + 1) % 8
        else: # defensive programming
            print( "uh oh... direction should *always* be either True or False" )
            cleanup()
            exit( 1 )
        time.sleep( sleep )

    lowOutputPin()
    pub.publish(True)
    exit( 0 )

def setSpeed(data):
    max_step_sleep = 0.002
    min_step_sleep = 0.001

    sleep = max_step_sleep + (float(data.data - 1) * ((min_step_sleep - max_step_sleep)/99.0))
    setStepSleep(sleep)
    print(sleep)

def listener():
    rospy.init_node('rotating_table', anonymous=True)
    rospy.on_shutdown(cleanup)
    rospy.Subscriber('cmd_rotating_table', String, rotate)
    rospy.Subscriber('slowdown_error', Int16, setSpeed)
    rospy.spin()

if __name__ == "__main__":
    for av in sys.argv[1:]:
        errorType = av[:2]
        if errorType == "-c":
            try:
                nComponentError = int(av[2]) - 1
                componentError = True
            except:
                print("Error: no number in the argument")
    setStepSleep(0.001)
    listener()