#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int16
import RPi.GPIO as GPIO
import time
import threading, sys

#--------- GLOBAL VARIABLE ---------#
# Publishers needed for communicating with the production line node.
pub = rospy.Publisher('rs_rotating_table', String, queue_size=10)

# Variables used to generate errors at the specified time
# during the node launch.
slowdownError = False
nSlowdownError = 0
counterSlowdownError = 0
componentError = False
nComponentError = 0
counterComponentError = 0

# Lock variable for all the global variables that are shared across multiple threads
# and might be accessed at the same time.
speedLock = threading.Lock()
componentErrorLock = threading.Lock()
slowdownErrorLock = threading.Lock()

# Initializing variable concenring the control of the stepper motor by the Raspberry Pi.

# Define how quick the rotatinga table is moving.
# Careful lowering this, mechanical limitation of the motor can create issue.
step_sleep = 0.001 # This value is near to minimum limit that can be reached.
# Step to execute a 180° rotation. Look at the README file to understad how to compute this value. 
step_count = 11605
# True for clockwise, False for counter-clockwise 
direction = False 

# Define stepper motor sequence.
step_sequence = [[1,0,0,1],
                 [1,0,0,0],
                 [1,1,0,0],
                 [0,1,0,0],
                 [0,1,1,0],
                 [0,0,1,0],
                 [0,0,1,1],
                 [0,0,0,1]]

# Define the GPIO pins connected to the input of the ULN2003 driver.
in1 = 17 # GPIO 17
in2 = 18 # GPIO 18
in3 = 27 # GPIO 27
in4 = 22 # GPIO 22

# Select the "Broadcom SOC channel" number mode to refer to the GPIO.
GPIO.setmode( GPIO.BCM )

# Set up the GPIO as pinout.
GPIO.setup( in1, GPIO.OUT )
GPIO.setup( in2, GPIO.OUT )
GPIO.setup( in3, GPIO.OUT )
GPIO.setup( in4, GPIO.OUT )

# Initialize the GPIO pins to a LOW state.
GPIO.output( in1, GPIO.LOW )
GPIO.output( in2, GPIO.LOW )
GPIO.output( in3, GPIO.LOW )
GPIO.output( in4, GPIO.LOW )

# Define the list of the motor pins.
motor_pins = [in1,in2,in3,in4]

#--------- FUNCTION ---------#
# Functions for getting and setting variables shared across multiple threads.
def getStepSleep():
    with speedLock :
        return step_sleep
    
def setStepSleep(sleep):
    with speedLock :
        global step_sleep
        step_sleep = sleep

def getCounterSlowdownError():
    with slowdownErrorLock:
        return counterSlowdownError
    
def addCounterSlowdownError():
    with slowdownErrorLock:
        global counterSlowdownError
        counterSlowdownError += 1

def getCounterComponentError():
    with componentErrorLock:
        return counterComponentError
    
def addCounterComponentError():
    with componentErrorLock:
        global counterComponentError
        counterComponentError += 1

# Function to execute the necessary cleanup operations.
def cleanup():
    lowOutputPin()
    GPIO.cleanup()

# Function to set the specified pins to a LOW state.
def lowOutputPin():
    GPIO.output( in1, GPIO.LOW )
    GPIO.output( in2, GPIO.LOW )
    GPIO.output( in3, GPIO.LOW )
    GPIO.output( in4, GPIO.LOW )

# Function that controls the rotation of the rotating table.
def rotate(data):

    # Generate specified errors during node launch for the robot.
    if (slowdownError and nSlowdownError == getCounterSlowdownError()):
        addCounterSlowdownError()
        speed = 50
        pub.publish(str(speed))
        return
    elif (componentError and nComponentError == getCounterComponentError()):
        addCounterComponentError()
        pub.publish("OK")
        return

    addCounterSlowdownError()
    addCounterComponentError()

    # Start the rotation. 
    motor_step_counter = 0 # Used to apply the right step sequence.
    sleep = getStepSleep()
    for i in range(step_count):

        # Apply the correct state to the pins.
        for pin in range(0, len(motor_pins)):
            GPIO.output( motor_pins[pin], step_sequence[motor_step_counter][pin] )

        # Keep track of the correct step sequence to apply based on the rotating direction.
        if direction==True:
            motor_step_counter = (motor_step_counter - 1) % 8
        elif direction==False:
            motor_step_counter = (motor_step_counter + 1) % 8
        else:
            print( "uh oh... direction should *always* be either True or False" )
            cleanup()
            exit( 1 )

        # Wait to realize the next step.
        time.sleep( sleep )

    # When the rotation is completed, publish the message.
    lowOutputPin()
    pub.publish("OK")
    exit( 0 )

# Function to set the speed (and thus the step sleep) of the rotating table.
# Note: The speed changes in a linear way between two extremes.
def setSpeed(data):
    max_step_sleep = 0.002
    min_step_sleep = 0.001

    sleep = max_step_sleep + (float(data.data - 1) * ((min_step_sleep - max_step_sleep)/99.0))
    setStepSleep(sleep)

# Function that defines the ROS node and sets up subscribers for communication with
# the production line node and the FlexBE behavior "Production Line".  
def listener():
    rospy.init_node('rotating_table', anonymous=True)
    rospy.on_shutdown(cleanup)
    rospy.Subscriber('cmd_rotating_table', String, rotate)
    rospy.Subscriber('slowdown_error', Int16, setSpeed)
    rospy.spin()

if __name__ == "__main__":

    # When launching the node, arguments can be used to define the type
    # of error to generate and when to generate it.
    # s -> slowdown error
    # c -> component not arrived error
    for av in sys.argv[1:]:
        errorType = av[:2]
        if errorType == "-s":
            try:
                nSlowdownError = int(av[2]) - 1
                slowdownError = True
            except:
                print("Error: no number in the argument")
        elif errorType == "-c":
            try:
                nComponentError = int(av[2]) - 1
                componentError = True
            except:
                print("Error: no number in the argument")

    # Set the rotating table speed to its maximum.
    setStepSleep(0.001)

    # Initialize the ROS node and start the subscribers.
    listener()