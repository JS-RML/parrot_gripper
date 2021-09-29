import time
from math import *
import matplotlib.pyplot as plt
import odrive
from odrive.enums import *
print("finding an odrive...")
od = odrive.find_any()
print("Odrive Voltage: " + str(od.vbus_voltage))
axis0 = od.axis0
motor0 = axis0.motor
encoder0 = axis0.encoder
controller0 = axis0.controller
axis1 = od.axis1
motor1 = axis1.motor
encoder1 = axis1.encoder
controller1 = axis1.controller

#encoder reading at zero joint angle
encoder0_zero = 0.015
encoder1_zero = 0.07
def get_angle():
    return (encoder0.pos_estimate - encoder0_zero) * 360, 180 - (encoder1.pos_estimate - encoder1_zero) * 360

def write_angle(motor_angle):
    q1, q2 = motor_angle
    raw_q1 = q1/360 + encoder0_zero
    raw_q2 = (180-q2)/360 + encoder1_zero
    #print("write angle:"+str((raw_q1,raw_q2)))
    axis0.controller.input_pos = raw_q1
    axis1.controller.input_pos = raw_q2
    
def set_stiffness(stiffness):
    stiffness = max(30,min(stiffness, 150)) #within 30-150 range
    controller0.config.pos_gain = stiffness
    controller1.config.pos_gain = stiffness
    
def set_soft():
    set_stiffness(15)

def set_hard():
    set_stiffness(150)
    
#position filter
def set_pos_filter(bandwidth):
    controller0.config.input_mode = INPUT_MODE_POS_FILTER
    controller1.config.input_mode = INPUT_MODE_POS_FILTER
    controller0.config.input_filter_bandwidth = bandwidth
    controller1.config.input_filter_bandwidth = bandwidth
    
#direct position input
def set_pos_raw_input():
    controller0.config.input_mode = INPUT_MODE_PASSTHROUGH
    controller1.config.input_mode = INPUT_MODE_PASSTHROUGH

setpoint_rate = 60 #rate of setting motor input
def enable_motor(stiffness=70, bandwidth = 50):
    print("enabling motor control...")
    set_pos_filter(bandwidth)
    set_stiffness(stiffness)
    axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

def disable_motor():
    print("disabling motor control...")
    axis0.requested_state = AXIS_STATE_IDLE
    axis1.requested_state = AXIS_STATE_IDLE

l1 = 92.5 #length of proximal link: 91.77mm in solidworks
l2 = 118 #legnth of distal link: 117mm in solidworks
def forward_kinematics(motor_angle):
    q1, q2 = motor_angle
    alpha = (q1+q2)/2
    beta = (q2-q1)/2
    r = l1*cos(radians(beta)) + sqrt(l2**2 - (l1*sin(radians(beta)))**2)
    x = r*cos(radians(alpha))
    y = r*sin(radians(alpha))
    return x, y

def inverse_kinematics(pos):
    x, y = pos
    r = sqrt(x**2+y**2)
    r = max(35,min(r, l1 + l2 - 5))
    alpha = degrees(atan2(y,x))
    beta = degrees(acos((l2**2-l1**2-r**2)/(-2*l1*r)))
    return alpha-beta, alpha+beta

def home():
    print("homing...")
#     enable_motor()
    controller0.input_pos = encoder0_zero
    controller1.input_pos = encoder1_zero
    
def get_pos():
    return forward_kinematics(get_angle())
    
def goto(pos, T = 0):
    if T == 0:
        write_angle(inverse_kinematics(pos))
    else:
        x_start,y_start = get_pos()
        x_goal,y_goal = pos
        t = 0
        t_start = time.time()
        while t<T:
            x = x_start + (x_goal-x_start)*t/T
            y = y_start + (y_goal-y_start)*t/T
            write_angle(inverse_kinematics((x,y)))
            t = time.time() - t_start
            time.sleep(1/setpoint_rate)
    
def rotate(point, center, angle, T):
    p_x, p_y = point
    c_x, c_y = center
    ang_rad = radians(angle)
    goto(point)
    t = 0
    t_start = time.time()
    x = 0
    y = 0
    while t<T:
        x = cos(ang_rad*t/T) * (p_x-c_x) - sin(ang_rad*t/T) * (p_y-c_y) + c_x
        y = sin(ang_rad*t/T) * (p_x-c_x) + cos(ang_rad*t/T) * (p_y-c_y) + c_y
        #print(x,y)
        goto((x,y))
        t = time.time() - t_start
        time.sleep(1/setpoint_rate)
    print("Rotated to: " + str((x,y)))
    return (x,y)

gripper_link = 100 #mm
gripper_finger_offset = (1,7) #offset from finger joint to finger corner
gripper_joint_width = 25.4 #width of two joint at the gripper base
joint2palm = 9.38 #vertical offset of gripper base joint to palm's origin
finger_thickness = 9.85
tip_offset = (40,80) #offset from finger corner to tip
def get_tip_pos(gripper_width, finger_direction): 
    #gripper_width in meter
    #finger_direction = -1: back finger
    gripper_width = gripper_width*1000
    joint_angle = asin((gripper_width/2 + finger_thickness + gripper_finger_offset[0] - gripper_joint_width/2)/gripper_link)
    y_tip = gripper_link*cos(joint_angle) + gripper_finger_offset[1] + tip_offset[1] - joint2palm
    x_tip = finger_direction*(gripper_width/2 + finger_thickness - tip_offset[0])
    return (x_tip, y_tip)