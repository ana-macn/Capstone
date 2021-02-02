# Glove Input v 1.0
import serial
from serial.tools import list_ports


def print_hi(name):
    print("Hello ", name)  # Press Ctrl+F8 to toggle the breakpoint.


""" def connect_glove():
    # Find COM port - Glove
    all_devices = list_ports.comports()
    for device in all_devices:
        print("Device: ", device.name, device.usb_info(), device.device, device.description)
        if "USBUART - CDC" in device.description:
            serial_connection = serial.Serial(port=device.device, baudrate=9600, bytesize=8, timeout=2,
                                              stopbits=serial.STOPBITS_ONE)
            return serial_connection
            break """

class AllSensors:
    pinky_mcp = 0
    ring_mcp = 0
    middle_mcp = 0
    index_mcp = 0
    thumb_mcp = 0


def connect_glove():
    # Find COM port - Glove
    all_devices = list_ports.comports()
    for device in all_devices:
        print("Device: ", device.name, device.usb_info(), device.device, device.description)
        if "USB" in device.description:
            serial_connection = serial.Serial(port=device.device, baudrate=9600, bytesize=8, timeout=1,
                                                stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE)
            return serial_connection
            break

def separe_lectures(lecture):
    all_lectures = (lecture[4:len(lecture)].replace("]", "").split("["))[1:17]
    parsed_lectures = [0]*17

    for element in all_lectures:
        values = element.split(":")
        if isinstance(values[0], str) : 
            parsed_lectures[ int(values[0]) ] = values[1]
        
    #print(parsed_lectures)
    return parsed_lectures

def print_per_finger(lectures):
    print(lectures[8])
    """ print(
       "|", lectures[0], "    |    ", lectures[1], "    |    ", lectures[2], "    |    ",
        lectures[3], "    |    ", lectures[4], "    |    ", lectures[5], "    |    ",
        lectures[6], "    |    ", lectures[7], "     |   ", lectures[8], "    |    ",
        lectures[9], "    |") """

    
    if int(lectures[6]) > 10000 : 
        txt='{"movement":"forward"}'
        file_ana=open("//wsl$/Ubuntu/home/varunik/data_test.txt",'w')
        file_ana.write(txt)
        file_ana.close()
        pass
    else:
        txt='{"movement":"none"}'
        file_ana=open("//wsl$/Ubuntu/home/varunik/data_test.txt",'w')
        file_ana.write(txt)
        file_ana.close()
        pass
    

    # print("Pinky MCP", lectures[0])
    # print("Pinky PIP", lectures[1])
    # print("Ring MCP", lectures[2])
    # print("Ring PIP", lectures[3])
    # print("Middle MCP", lectures[4])
    # print("Middle PIP", lectures[5])
    # print("Index MCP", lectures[6])
    # print("Index PIP", lectures[7])
    # print("Thumb MCP", lectures[8])
    # print("Thumb PIP", lectures[9])


if __name__ == '__main__':

    # Finding Glove port and connect
    glove = connect_glove()
    """ glove.write("enable\n".encode())
    print('close')
    serial.time.sleep(2)
    glove.write("c\n".encode())
    print('open')
    serial.time.sleep(2)
    glove.write("o\n".encode()) """
    print('starting')
    serial.time.sleep(2)
    
    # Printing lecture
    while True:
        try:
            sensors_lectures = separe_lectures(glove.readline().decode())
            print_per_finger(sensors_lectures)
        except KeyboardInterrupt:
            if glove.isOpen():
                glove.close()
                print("Connection closed!")
