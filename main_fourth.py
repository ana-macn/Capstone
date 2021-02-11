import speech_recognition as speechRec
import serial
from serial.tools import list_ports
import pyttsx3

#Globals
# file_path = "//wsl$/Ubuntu/home/anamacn/data_test.txt"
file_path = '//wsl$/Ubuntu/home/varunik/data_test.txt'
glove_on = True
last_gesture = "none"
#To make the system talk
engine = pyttsx3.init()

#Functions
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

def voice_recognition(voice):
    print('Starting recording...')
    with speechRec.Microphone() as source:
        voice.adjust_for_ambient_noise(source)
        data = voice.record(source, duration=3)
        print(data)
        text = voice.recognize_google(data, language='en')
        return text



if __name__ == '__main__':
    print('Sofwerx Drone Control - The Best Group')

    # Finding Glove port, connect and enable reading the lectures
    glove = connect_glove()
    glove.write("enable\n".encode())
    serial.time.sleep(1)
    print('Starting glove mode...')

    while True:
        # Reading glove data
        sensors_lectures = separe_lectures(glove.readline().decode())

        # Finding is finger is closed/open
        fingers_positions = finger_analysis(sensors_lectures)

        # Detecting gesture
        gesture = identify_gesture(fingers_positions)
        if gesture == "voice":
            glove_on = False
        if gesture == "glove":
            glove_on = True
        if gesture != last_gesture:
            last_gesture = gesture

            if glove_on:
                print(gesture, last_gesture)
                try:
                    txt =  '{"gesture":"' + gesture + '"}'
                    file = open(file_path, "w")
                    file.write(txt)
                    file.close()
                    pass
                except KeyboardInterrupt:
                    if glove.isOpen():
                        glove.close()
                        print("Connection closed!")
            else:
                print(gesture, "in voice")
                if gesture == 'rocker':
                    print("Recording voice ...")
                    voice = speechRec.Recognizer()
                    command = voice_recognition(voice)
                    print('Word recognized: ', command)
                    print('\n')
                    txt = '{"gesture":"' + command + '"}'
                    file = open(file_path, "w")
                    file.write(txt)
                    file.close()
                    pass
