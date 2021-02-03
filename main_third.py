# Glove Input v 1.0
import serial
from serial.tools import list_ports

def connect_glove():
    # Find COM port - Glove
    all_devices = list_ports.comports()
    for device in all_devices:
        print("Device: ", device.name, device.usb_info(), device.device, device.description)
        if "USB" in device.description:
            serial_connection = serial.Serial(port=device.device, baudrate=9600, bytesize=8, timeout=2,
                                              stopbits=serial.STOPBITS_ONE)
            return serial_connection
            break

def separe_lectures(lecture):
    all_lectures = (lecture[4:len(lecture)].replace("]", "").split("["))[1:17]
    parsed_lectures = [None]*17

    for element in all_lectures:
        parsed_lectures[ int((element.split(":"))[0]) ] = (element.split(":"))[1]
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

    # print("Pinky MCP", lectures.pinky_mcp)
    # print("Pinky PIP", lectures.pinky_pip)
    # print("Ring MCP", lectures.ring_mcp)
    # print("Ring PIP", lectures.ring_pip)
    # print("Middle MCP", lectures.middle_mcp)
    # print("Middle PIP", lectures.middle_pip)
    # print("Index MCP", lectures.index_mcp)
    # print("Index PIP", lectures.index_pip)
    # print("Thumb MCP", lectures.thumb_mcp)
    # print("Thumb PIP", lectures.thumb_pip)

def finger_analysis(sensors_lectures):
    fingers = []
    for x in range(1, 10, 2):
        if isinstance(sensors_lectures[x], str):
            if int(sensors_lectures[x]) > 10000:
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
        return '{"gesture": "fist"}' ## stop
    elif closed == 0:
        return '{"gesture": "open"}' ## hover
    elif closed == 3:
        if fingers[1] == "closed" and fingers[2] == "closed" and fingers[3] == "closed":
            return '{"gesture": "rocker"}'  ## patrol
        elif fingers[1] == "closed" and fingers[2] == "closed" and fingers[0] == "closed":
            return '{"gesture": "gun"}' ## way point 1
        else:
            return '{"gesture": "keep"}'
    elif closed == 1: 
        if fingers[3] == "closed":
            return '{"gesture": "toggle"}' ## Enable/Toggle the glove/headset operation
        else:
            return '{"gesture": "keep"}'
    else:
        return '{"gesture": "keep"}'



if __name__ == '__main__':
    print('Sofwerx Drone Control - The Best Group')
    # Finding Glove port, connect and enable reading the lectures

    glove = connect_glove()
    glove.write("enable\n".encode())
    print('MENÚ:  1) Start System  2) Turn off System')
    option = int(input("Select your option:  "))
    print(option)
    
    while option == 1 :
        try:
            sensors_lectures = separe_lectures(glove.readline().decode())
            
            # Finding is finger is closed/open
            fingers_positions = finger_analysis(sensors_lectures)

            # Detecting gesture
            gesture = identify_gesture(fingers_positions)
            print(gesture)

            if gesture == "toggle":
                option = 2
            else:
                txt = gesture
                file = open("//wsl$/Ubuntu/home/varunik/data_test.txt", "w")
                file.write(txt)
                file.close()

                pass

            

            
        except KeyboardInterrupt:
            if glove.isOpen():
                glove.close()
                print("Connection closed!")
    
    if option != 1 :
        if glove.isOpen():
            glove.close()
            print("Connection closed!")
