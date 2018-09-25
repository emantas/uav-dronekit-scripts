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



vehicle = copterConnection()
vehicle.parameters['BATT_CAPACITY']=999999999
print(" Global Location: %s" % vehicle.location.global_frame)
# AUTO MISSION MAVLINK Commands
# Command Template
# Command(0,0,0,FrameofReference,MAVLinkCommand,CurrentWP,AutoContinue,param1,param2,param3,param4,param5,param6,param7)

home = vehicle.location.global_relative_frame
wp0 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, home.lat, home.lon ,home.alt)
wp1 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.877143, -9.432208 , 20)
wp2 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.872546, -9.413497 ,20)
wp3 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.853245, -9.449031 ,20)
wp4 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.802599 , -9.477037 ,25)
wp5 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0)

waypoints = vehicle.commands
waypoints.download()
waypoints.wait_ready()
waypoints.clear()

waypoints.add(wp0)
waypoints.add(wp1)
waypoints.add(wp2)
waypoints.add(wp3)
waypoints.add(wp4)
waypoints.add(wp5)

vehicle.commands.upload()
armTakeoff(10)
vehicle.airspeed = 7
vehicle.mode = VehicleMode("AUTO")
while not vehicle.mode == "AUTO":
    time.sleep(.2)

while vehicle.location.global_relative_frame.alt>2:
    print("Mission is being executed")
    time.sleep(2)
