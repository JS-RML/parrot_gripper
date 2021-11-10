import time
import serial
import signal
from math import *
import numpy as np
from scipy import interpolate
import scipy.io as sio

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
set_serial(serial_index = 1)

global grip
def set_gripper(obj):
    global grip
    grip = obj
    
def myHandler(signum, frame):   
       pass

def send_str(msg): #msg:string 
    global ser
    try:
        global ser
        signal.signal(signal.SIGALRM, myHandler)
        signal.setitimer(signal.ITIMER_REAL, 0.001)
        ser.write(str.encode(msg))
        print('sending msg:', msg)
        signal.setitimer(signal.ITIMER_REAL, 0)
    except:
        print('reconnect arduino')
        set_serial(serial_index = 0)
        send_str(msg)   
        
def set_servo_angle(f1_ang,f2_ang):
    send_str("a" + str(np.round(f1_ang,2))+ "b" + str(np.round(f2_ang,2)))

# mapping from Q1_ to gripper command
grip_cmd = np.array([0, 25, 50, 75, 100, 125, 150, 175, 200, 225])
Q1_input= np.array([97.2,95,87,81.6,76.5,72.6,67,63.1,57.5,54])
ang2grip = interpolate.interp1d(Q1_input, grip_cmd, 'slinear')
# mapping from link21 length to servo angle
s1_angle = np.array([0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65]) #servo1 input angle 
s2_angle = 180 - s1_angle #servo2 input angle
link21_s1 = np.array([100,100.5,101.5,103,104,105,106,107,108,109,110,111,112,112.5]) #length of virtual link (mm)
link21_s2 = np.array([100,100.5,101,102,103,104,105,106,107,108,109,110.5,111.5,112])
L2s1 = interpolate.interp1d(link21_s1, s1_angle, 'linear')
L2s2 = interpolate.interp1d(link21_s2, s2_angle, 'linear')
# load IK map
ik_map = sio.loadmat('fivebar_finger_ik.mat')['ft2joint']
# gripper linkages parameters
L1 = 31.75
L2 = 86.4889215
def set_tip_dist(dx, dy):
    if dy >= 0:
        finger2move = 0
    else:
        dy = -dy
        finger2move = 1
    q1 = interpolate.griddata(ik_map[:,:2],ik_map[:,2],(dx,dy),method='linear')
    q21 = interpolate.griddata(ik_map[:,:2],ik_map[:,3],(dx,dy),method='linear')
    l21 = sqrt(L1**2+L2**2-2*L1*L2*cos(radians(q21))); # compute Link21 length
    print ('linkage variable:',(q1,q21,l21))
    grip.gripper_action(int(ang2grip(q1)))
    print('gripper_cmd:', ang2grip(q1))
    if finger2move == 0:
        set_servo_angle(L2s1(100),L2s2(l21))
    else:
        set_servo_angle(L2s1(l21),L2s2(100))
        
def tip_run_arc(radius, angle, direction, step = 1):
    for i in range(0,angle+1,step):
        x = radius * cos(radians(i));
        y = radius * sin(radians(i));
        if direction != 0:
            y = -y
        set_tip_dist(x,y)
    
    
if __name__ == "__main__":
    while True:
        try:
            cmd = input("Enter motor angle (0-180):")
            set_angle(cmd)
        except:
            break