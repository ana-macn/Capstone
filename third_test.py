from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import math
# To read the incoming txt values
import json
import time

#global variables

duration = 4
ground_speed = 4

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

#Functions 
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

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

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def send_ned_velocity(vehicle,vx, vy, vz, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.
    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        vx, vy, vz, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        #time.sleep(1)

# Functions to calculate distance
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
            print("Reached target")
            break
        time.sleep(2)
## MAIN CODE

#MANUAL MANIPULATION
print("Vehicle starts being manipulated by the glove")
command = "open"
ground = True
while  True :
    ## Read data from .txt file goes here
    file_ana=open('data_test.txt')
    command = "none"
    data=file_ana.read()
    
    if data :
       
        json_data=json.loads(data)
        #  print(json_data)

        ## Moving the drone
        if json_data: 

            if ground :

                if json_data["gesture"] == "bulls":
                    #vehicle.mode = VehicleMode("GUIDED")
                    print("Taking off")
                    ## Elevate drone 5mts 
                    arm_and_takeoff(5)
                    ground = False
            else:
                # print(command, json_data["gesture"])
                if json_data["gesture"] != command:
                    # print("Vehicle starts going to", json_data["gesture"])
                    command = json_data["gesture"]

                    if json_data["gesture"] == "open":  #Forward
                        #print("hover")
                        #vehicle.mode = VehicleMode("LOITER")
                        vehicle.mode = VehicleMode("GUIDED")
                        print("going forward")
                        send_ned_velocity(vehicle, ground_speed, 0, 0, duration)

                    elif json_data["gesture"] == "fist":  # Backwards
                        vehicle.mode = VehicleMode("GUIDED")
                        print("going backward")
                        send_ned_velocity(vehicle, -ground_speed, 0, 0, duration)
                        
                    elif json_data["gesture"] == "rocker": #Right
                        vehicle.mode = VehicleMode("GUIDED")
                        print("going to the right")
                        send_ned_velocity(vehicle, 0, ground_speed, 0, duration)
                        
                    elif json_data["gesture"] == "gun": #Left
                        vehicle.mode = VehicleMode("GUIDED")
                        print("going to the left")
                        send_ned_velocity(vehicle, 0, -ground_speed, 0, duration)
                    elif json_data["gesture"] == "keep":
                        print("doing nothing")

                    elif json_data["gesture"] == "land":
                        print("Landing started")
                        vehicle.mode = VehicleMode("LAND")
                        ground = True
                        # print("Close vehicle object")
                        # vehicle.close()

                    elif json_data["gesture"] == "home": #Go home
                        print("return to home")
                        vehicle.mode = VehicleMode("RTL")
                    
                    elif json_data["gesture"] == "Patrol":
                        print("wait for patrol")

                    elif json_data["gesture"] == "number one":  # Go to Waypoint 1
                        vehicle.mode = VehicleMode("GUIDED")
                        print('Going to the Yuengling')
                        point1 = LocationGlobalRelative(28.0579597,-82.4048829, 10)
                        vehicle.simple_goto(point1)

                    elif json_data["gesture"] == "number two":  # Go to Waypoint 2
                        vehicle.mode = VehicleMode("GUIDED")
                        print('Going to the Marhsall Student')
                        point2 = LocationGlobalRelative(28.0642702, -82.4113123, 10)
                        vehicle.simple_goto(point2)

                    
                        ## Shut down simulator if it was started.
                    """ if sitl:
                        sitl.stop() 
                        break """
                    time.sleep(duration)
                else:
                    print(" hello ")


# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop() 
