import time
import serial
import signal
from math import *

global ser
def set_serial(serial_index = 0):
    global ser
    baudRate = 57600
    print ('trying /dev/ttyACM',serial_index)
    serialPort = "/dev/ttyACM" + str(serial_index)
    try:
        global ser
        ser = serial.Serial(serialPort, baudRate, timeout=0.5)
        time.sleep(0.5)
    except:
        if serial_index >10:
            set_serial(serial_index=0)
        else:
            time.sleep(0.5)
            set_serial(serial_index + 1)

# serialPort = "/dev/ttyACM0"
# baudRate = 57600
# ser = serial.Serial(serialPort, baudRate, timeout=0.5)
set_serial(serial_index = 0)

def myHandler(signum, frame):   
       pass

def set_angle(ang): # ang:0-180 in string
    print('setting angle:',int(ang),'...')
    global ser
    try:
        global ser
        signal.signal(signal.SIGALRM, myHandler)
        signal.setitimer(signal.ITIMER_REAL, 0.001)
        ser.write(str.encode(ang))
        signal.setitimer(signal.ITIMER_REAL, 0)
    except:
        print('reconnect arduino')
        set_serial(serial_index = 0)
        set_angle(ang)

def set_offset(offset): #mm:+ve:extend, -ve:retract
    angle = 90 + (180-0)/(14.5-(-15)) * offset
    set_angle(str(int(angle)))
    
gripper_link = 100 #mm
gripper_finger_offset = (1,7) #offset from finger joint to finger corner
gripper_joint_width = 25.4 #width of two joint at the gripper base
joint2palm = 9.38 #vertical offset of gripper base joint to palm's origin
finger_thickness = 9.85
tip_offset = (50,80) #offset from finger corner to tip
def get_tip_pos(gripper_width, finger_id):
    ''' Get fingertip position in palm's frame in mm
        gripper_width: gripper set width in meter
        finger_id: 1:front finger, 2:back finger(at the side of gripper's cable)
    '''
    gripper_width = gripper_width*1000
    joint_angle = asin((gripper_width/2 + finger_thickness + gripper_finger_offset[0] - gripper_joint_width/2)/gripper_link)
    y_tip = gripper_link*cos(joint_angle) + gripper_finger_offset[1] + tip_offset[1] - joint2palm
    x_tip = finger_id*(gripper_width/2 + finger_thickness - tip_offset[0])
    return (x_tip, y_tip)

if __name__ == "__main__":
    while True:
        try:
            #cmd = input("Enter motor angle (0-180):")
            #set_angle(cmd)
            cmd = input("Enter offset (mm):")
            set_offset(float(cmd))
        except:
            break