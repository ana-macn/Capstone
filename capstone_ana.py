# Inputs
import speech_recognition as speechRec
import serial
from serial.tools import list_ports
import time
import pyttsx3
# Drone
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import math
import argparse

# Other tools
import json
import time

#Globals
glove_on = True
last_gesture = "none"
voice_commands = ["home","number one","number two", "patrol", "land", "glove"]
duration = 10
ground_speed = 3
#To make the system talk
narrator = pyttsx3.init()

# Functions for the inputs
def connect_glove():
    # Find COM port - Glove
    all_devices = list_ports.comports()
    for device in all_devices:
        print("Device: ", device.name, device.usb_info(), device.device, device.description)
        if "4" in device.name:
            print(" in ")
            serial_connection = serial.Serial(port=device.device, baudrate=9600, bytesize=8, 
                                              timeout=12, stopbits=serial.STOPBITS_ONE)
            print("connected")
            return serial_connection
            break

def separe_lectures(lecture):
    print("lecture", lecture.count(":"))
    all_lectures = (lecture[4:len(lecture)].replace("]", "").split("["))[1:17]
    parsed_lectures = [None] * 17

    for element in all_lectures:
        parsed_lectures[int((element.split(":"))[0])] = (element.split(":"))[1]
    return parsed_lectures

def print_per_finger(lectures):
    print(
        "__________________________________________________________________________________________________________________________")
    print(
        "| Pinky MCP | Pinky PIP | Ring MPC | Ring PIP | Middle MCP | MIddle PIP | Index MCP | Index PIP | Thumb MCP | Thumb PIP |")
    print(
        "__________________________________________________________________________________________________________________________")
    print(
        "|", lectures[0], "    |    ", lectures[1], "    |    ", lectures[2], "    |    ",
        lectures[3], "    |    ", lectures[4], "    |    ", lectures[5], "    |    ",
        lectures[6], "    |    ", lectures[7], "     |   ", lectures[8], "    |    ",
        lectures[9], "    |")

def finger_analysis(sensors_lectures):
    fingers = []
    for x in range(1, 10, 2):
        if isinstance(sensors_lectures[x], str):
            if int(sensors_lectures[x]) > 7000:
                fingers.append("closed")
            else:
                fingers.append("open")
    return fingers

def identify_gesture(fingers):
    closed = 0
    for x in fingers:
        if x == "closed":
            closed = closed + 1
    if closed == 5:
        return "fist"
    elif closed == 0:
        return "open"
    elif closed == 2:
        if fingers[1] == "closed" and fingers[0] == "closed":
            return "glove"
        else:
            return "keep"
    elif closed == 3:
        if fingers[1] == "closed" and fingers[2] == "closed" and fingers[3] == "closed":
            return "rocker"
        if fingers[1] == "closed" and fingers[2] == "closed" and fingers[4] == "closed":
            return "bulls"
        elif fingers[1] == "closed" and fingers[2] == "closed" and fingers[0] == "closed":
            return "gun"
        else:
            return "keep"
    elif closed == 1:
        if fingers[3] == "closed":
            return "voice"
        else:
            return "keep"
    else:
        return "keep"

def voice_recognition(voice, place):
    print('Listening...', place)
    with speechRec.Microphone() as source:
        voice.adjust_for_ambient_noise(source)
        data = voice.record(source, duration=3)
        try:
            text = voice.recognize_google(data, language='en', show_all=True)
            if len(text) > 0:
                return text['alternative'][0]['transcript']
        except LookupError or UnknownValueError:  # speech is unintelligible
            pass

# Functions for the drone
def connect_drone():
    # Set up option parsing to get connection string
    
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
    return [vehicle, sitl]

def arming_vehicle(aTargetAltitude, vehicle):
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


if __name__ == '__main__':
    print('Sofwerx Drone Control - The Best Group')

    # Start Drone and SITL
    drone_params = connect_drone()
    drone = drone_params[0]
    sitl = drone_params[1]

    # Finding Glove port, connect and enable reading the lectures
    glove = connect_glove()
    # glove.write("enable\n".encode())
    time.sleep(1)

    print('Starting glove mode...')
    active = True
    glove = True
    while active:
        try:
            # Reading glove data
            time.sleep(0.5)
            glove.flushInput()
            if glove.readline().count(":") == 17 :
                sensors_lectures = separe_lectures(glove.readline())

                # Finding is finger is closed/open
                fingers_positions = finger_analysis(sensors_lectures)

                # Detecting gesture
                gesture = identify_gesture(fingers_positions)
                print(gesture)

                # Printing
            while glove:
                if gesture == "bulls": #Takeoff
                    arming_vehicle(3, drone)
                    time.sleep(3)
                elif gesture == "open":  #Hover
                    vehicle.mode = VehicleMode("LOITER")
                    print("I'm hovering")
                elif gesture == "gun": #Left
                    vehicle.mode = VehicleMode("GUIDED")
                    print("going to the left")
                    send_ned_velocity(vehicle, 0, -ground_speed, 0, duration)
                elif gesture == "rocker": #Right
                    vehicle.mode = VehicleMode("GUIDED")
                    print("going to the right")
                    send_ned_velocity(vehicle, 0, ground_speed, 0, duration)
                elif gesture == "fist":  # Backwards
                    vehicle.mode = VehicleMode("GUIDED")
                    print("going backwards")
                    send_ned_velocity(vehicle, -ground_speed, 0, 0, duration)
                elif gesture == "voice": #Voice Control Begins
                     v_command = True
                     glove = False
                     while v_command:
                        voice = speechRec.Recognizer()
                        command = voice_recognition(voice, "general")
                        if isinstance(command, str) and command in voice_commands:
                            narrator.say("Executing command", command) # Like "Hey Siri!"
                            narrator.runAndWait()
                            print("Executing command", command)

                            elif command == "home": #Go home
                                print("Returning to Launching Point (Home)")
                                vehicle.mode = VehicleMode("RTL")
                    
                            elif command == "Patrol":
                                print("I'm patroling")
                                heading = 360 # Degrees the drone will turn on itself
                                condition_yaw(heading, relative=True)

                            elif command == "number one":  # Go to Waypoint 1
                                vehicle.mode = VehicleMode("GUIDED")
                                print('Going to the Yuengling')
                                point1 = LocationGlobalRelative(28.0579597,-82.4048829, 10)
                                vehicle.simple_goto(point1)

                            elif command == "number two":  # Go to Waypoint 2
                                vehicle.mode = VehicleMode("GUIDED")
                                print('Going to the Marhsall Student')
                                point2 = LocationGlobalRelative(28.0642702, -82.4113123, 10)
                                vehicle.simple_goto(point2)
                            

                            elif command == "land":
                                print("Now landing....")
                                drone.mode = VehicleMode("LAND")

                                # Close vehicle object before exiting script
                                print("Close vehicle object")
                                drone.close()
                                active = False

                            elif command == "glove":  #Return to Glove Control
                                print("Going Back to Glove!")
                                v_command = False
                                glove = True
                    

        except KeyboardInterrupt:
                if glove.isOpen():
                    glove.close()
                    print("Connection closed!")


# Shut down simulator if it was started.
    if sitl:
        sitl.stop()

    
