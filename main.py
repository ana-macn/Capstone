import serial
from serial.tools import list_ports


def print_hi(name):
    print("Hello ", name)  # Press Ctrl+F8 to toggle the breakpoint.


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


class AllSensors:
    pinky_mcp = 0
    ring_mcp = 0
    middle_mcp = 0
    index_mcp = 0
    thumb_mcp = 0
    pinky_pip = 0
    ring_pip = 0
    middle_pip = 0
    index_pip = 0
    thumb_pip = 0


def separe_lectures(lecture):
    # print(lecture)
    all_lectures = (lecture[4:len(lecture)].replace("]", "").split("["))[1:17]
    parsed_lectures = [None]*17

    for element in all_lectures:
        parsed_lectures[ int((element.split(":"))[0]) ] = (element.split(":"))[1]
    return parsed_lectures


def print_per_finger(lectures):
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


if __name__ == '__main__':
    print_hi('Drone Control Group 1 - The best!')

    # Finding Glove port and connect
    glove = connect_glove()
    glove.write("enable\n".encode())
    print_hi('starting')

    print(
        "__________________________________________________________________________________________________________________________")
    print(
        "| Pinky MCP | Pinky PIP | Ring MPC | Ring PIP | Middle MCP | MIddle PIP | Index MCP | Index PIP | Thumb MCP | Thumb PIP |")
    print(
        "__________________________________________________________________________________________________________________________")
    # Printing lecture
    while True:
        try:
            sensors_lectures = separe_lectures(glove.readline().decode())
            print_per_finger(sensors_lectures)
            # serial.time.sleep(3)
            print("\n\n")
            # print(sensors_lectures)
        except KeyboardInterrupt:
            if glove.isOpen():
                glove.close()
                print("Connection closed!")
