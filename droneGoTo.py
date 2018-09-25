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

def getDistanceMeters(targetLocation, currentLocation):
    dLat = targetLocation.lat - currentLocation.lat
    dLon = targetLocation.lon - currentLocation.lon
    return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5    # hypotinousa


def flyAt(targetLocation):
    distanceToTargetLoc = getDistanceMeters(targetLocation, vehicle.location.global_relative_frame)
    vehicle.simple_goto(targetLocation)
    while vehicle.mode.name == "GUIDED":
        currentDistance = getDistanceMeters(targetLocation, vehicle.location.global_relative_frame)
        if currentDistance < distanceToTargetLoc*.01:
            print("Reached target waypoint")
            time.sleep(2)
            break
        time.sleep(1)
    return None


# Flight Scenario
vehicle = copterConnection()
vehicle.parameters['BATT_CAPACITY']=999999999
vehicle.airspeed = 7

# waypoint initialisation
wp1 = LocationGlobalRelative(38.877143, -9.432208 , 20)
wp2 = LocationGlobalRelative(38.872546, -9.413497 ,20)
wp3 = LocationGlobalRelative(38.853245, -9.449031 ,20)
wp4 = LocationGlobalRelative(38.802599 , -9.477037 ,25)

print(" Home Location: %s" % vehicle.location.global_frame)
armTakeoff(10)

while True:
    flyAt(wp1)
    flyAt(wp2)
    flyAt(wp3)
    flyAt(wp4)
