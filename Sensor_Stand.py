#python dronekit_test.py

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import time
import random
import math

currTime = time.ctime()

# Connect to the Vehicle (in this case a UDP endpoint)
vehicle = connect('127.0.0.1:14550', wait_ready=True)

#pymavlink with DroneKit API.

print "----Start of Sensor Test Stand process---"
vehicle.mode  = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.mode.name=='GUIDED' and not vehicle.armed:
        print " Starting test..."
        time.sleep(1)

#read data from Lidar, GPS, Airspeed

#listener for battery
@vehicle.on_message('BATTERY')
def listener(self,name,message):
    print 'voltage: %s' %message.voltage
    print 'current: %s' %message.current

#sending MAVLINK commands
'''
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
    time.sleep(2)
'''

#if key is pressed, finish test
vehicle.mode = VehicleModel("LAND")
vehicle.armed = False

#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()