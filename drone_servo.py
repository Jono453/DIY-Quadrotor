#Script to run on Pi with Mavlink + MissionPlanner (using Dronekit API)

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Vehicle
import time
import random
import math
from pymavlink import mavutil
import cv2
import argparse

#Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='UDP SITL Connection String')
parser.add_argument('--connect',
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None
FC_pin = 6
#UDPaddress = ''
#USB = 'com17'

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

    #send PWM signal to motor
def sendServo(pin,PWMvalue):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
        0, #confirmation
        pin,  # param 1, servo number
        PWMvalue,          # param 2, PWM (1000-2000)
        0, 0, 0, 0, 0) # param 3 ~ 7 not used
    #send command to vehicle
    vehicle.send_mavlink(msg)
    time.sleep(1)

#--Main function--

print("Connecting on: %s" % (connection_string,))
#print("Connecting to Emlid Edge on: %s" % (connection_string,))

vehicle = connect(connection_string, wait_ready=True)

#print "Arming motors: \n"
vehicle.mode  = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.mode.name == 'GUIDED' and not vehicle.armed: () #wait
print "Drone armed and ready\n"

while (True):

    sendServo(FC_pin,1500) #send servo pwm command to move arm to halfway point
    #print "Servo set to 1500"

    if cv2.waitKey(1) & 0xFF == ord('q'):
        sendServo(FC_pin,1000)
        break

vehicle.armed = False
#Close vehicle object before exiting script
print "Finished sensor test"
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()
