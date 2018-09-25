from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, Command
from pymavlink import mavutil
import time
import socket
import math
import argparse

def copterConnection():
    parser = argparse.ArgumentParser(description = 'Connect to Copter')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = args.connect

    # Start SITL if no connection string specified
    if not connection_string:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()
        print("Connected at : %s" %connection_string)

    # Connect to the Vehicle
    vehicle = connect(connection_string, wait_ready=True)
    return vehicle


def armTakeoff(targetAlt):
    # Pre-flight check & drone arm
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Ready to takeoff")
    vehicle.simple_takeoff(targetAlt)
    while True:
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= targetAlt * 0.97:
            print("Reached target altitude")
            break
        time.sleep(1)
    return None


def yawControl(degrees, relativeFrameID):
    if relativeFrameID:
        is_relative = 1 #yaw relative to the drove heading
    else:
        is_relative = 0 #yaw is absolute angle

    msg = vehicle.message_factory.command_long_encode(
       0, 0,                                        # target system , target component
       mavutil.mavlink.MAV_CMD_CONDITION_YAW,       # command type
       0,                                           # confirmation
       degrees,                                     # yaw degrees
       0,                                           # yaw speed
       1,                                           # direction -1:CCW   1:CW    (CW: ClockWise)
       is_relative,                                 # 1:relative offset    0:absolute value
       0, 0, 0)                                     # params not used in this msg

    vehicle.send_mavlink(msg)
    vehicle.flush()


def dummyYawInitialiser():
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    alt = vehicle.location.global_relative_frame.alt

    droneLocation = LocationGlobalRelative(lat, lon, alt)

    msg = vehicle.message_factory.set_position_target_global_int_encode( 0, 0, 0,
       mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
       0b0000111111111000,                                # type mask
       droneLocation.lat*1e7,
       droneLocation.lon*1e7,
       droneLocation.alt,
       0, 0, 0,                                           # x,y,z velocity in NED frame
       0, 0, 0,
       0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()



vehicle = copterConnection()

vehicle.parameters['BATT_CAPACITY']=99999999
dummyYawInitialiser()
vehicle.airspeed = 7
print(" Home Location: %s" % vehicle.location.global_frame)
armTakeoff(10)

time.sleep(2)
yawControl(30,1)
time.sleep(10)
yawControl(0,0)
time.sleep(10)
yawControl(270,1)
time.sleep(10)
print("End of Rotations")
