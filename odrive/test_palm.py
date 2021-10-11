import odrive
from odrive.enums import *
#from odrive.utils import start_liveplotter
from math import *
import time
import matplotlib.pyplot as plt
import numpy as np

#cursor control
import pyautogui

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

l1 = 92.5 #length of proximal link: 91.77mm in solidworks
l2 = 118 #legnth of distal link: 117mm in solidworks

#encoder reading at zero joint angle
encoder0_zero = 0.015
encoder1_zero = 0.07

setpoint_rate = 60 #rate of setting motor input
input_bandwidth = 50

import threading
from fibre.utils import Event
data_rate = 200
plot_rate = 20
num_samples = 500
def start_liveplotter(get_var_callback):
    """
    Starts a liveplotter.
    The variable that is plotted is retrieved from get_var_callback.
    This function returns immediately and the liveplotter quits when
    the user closes it.
    """

    cancellation_token = Event()

    global vals
    vals = []
    def fetch_data():
        global vals
        while not cancellation_token.is_set():
            try:
                data = get_var_callback()
            except Exception as ex:
                print(str(ex))
                time.sleep(1)
                continue
            vals.append(data)
            if len(vals) > num_samples:
                vals = vals[-num_samples:]
            time.sleep(1/data_rate)

    # TODO: use animation for better UI performance, see:
    # https://matplotlib.org/examples/animation/simple_anim.html
    def plot_data():
        global vals

        plt.ion()

        # Make sure the script terminates when the user closes the plotter
        def closed(evt):
            cancellation_token.set()
        fig = plt.figure()
        fig.canvas.mpl_connect('close_event', closed)

        while not cancellation_token.is_set():
            plt.clf()
            plt.plot(vals)
            plt.legend(list(range(len(vals))))
            fig.canvas.draw()
            fig.canvas.start_event_loop(1/plot_rate)

    fetch_t = threading.Thread(target=fetch_data)
    fetch_t.daemon = True
    fetch_t.start()
    
    plot_t = threading.Thread(target=plot_data)
    plot_t.daemon = True
    plot_t.start()

    return cancellation_token
    #plot_data()
# start_liveplotter(lambda: [encoder0.pos_estimate, encoder1.pos_estimate]) #may cause lag
# start_liveplotter(lambda:[odrv0.axis0.motor.current_meas_phB, odrv0.axis0.motor.current_meas_phC])

def get_angle():
    return (encoder0.pos_estimate - encoder0_zero) * 360, 180 - (encoder1.pos_estimate - encoder1_zero) * 360

def write_angle(motor_angle):
    q1, q2 = motor_angle
    raw_q1 = q1/360 + encoder0_zero
    raw_q2 = (180-q2)/360 + encoder1_zero
    #print("write angle:"+str((raw_q1,raw_q2)))
    controller0.input_pos = raw_q1
    controller1.input_pos = raw_q2

def enable_motor():
    print("enabling motor control...")
    controller0.config.input_mode = INPUT_MODE_POS_FILTER
    controller1.config.input_mode = INPUT_MODE_POS_FILTER
    controller0.config.input_filter_bandwidth = input_bandwidth
    controller1.config.input_filter_bandwidth = input_bandwidth
    axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

def disable_motor():
    print("disabling motor control...")
    axis0.requested_state = AXIS_STATE_IDLE
    axis1.requested_state = AXIS_STATE_IDLE

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
    print(r)
    r = max(35,min(r, l1 + l2 - 5))
    alpha = degrees(atan2(y,x))
    beta = degrees(acos((l2**2-l1**2-r**2)/(-2*l1*r)))
    return alpha-beta, alpha+beta

def set_soft():
    controller0.config.pos_gain = 30
    controller0.config.vel_gain = 1
    controller1.config.pos_gain = 30
    controller1.config.vel_gain = 1
    
def set_hard():
    controller0.config.pos_gain = 150
    controller0.config.vel_gain = 1
    controller1.config.pos_gain = 150
    controller1.config.vel_gain = 1

def main():
    while True:
        cmd = str(input("Enter command:")).lower()
        if cmd == 'd':
            disable_motor()
        elif cmd == 'e':
            enable_motor()
        elif cmd == 'home':
            print("homing...")
            enable_motor()
            controller0.input_pos = encoder0_zero
            controller1.input_pos = encoder1_zero
        elif cmd == 'plot_current':
            start_liveplotter(lambda: [motor0.current_meas_phB, motor1.current_meas_phB])
        elif cmd == 'cursor':
            print("homing...")
            set_soft()
            enable_motor()
            controller0.input_pos = encoder0_zero
            controller1.input_pos = encoder1_zero
            home_pos = forward_kinematics(get_angle())
            print('home pos:' + str(home_pos))
            init_pos =  pyautogui.position()
            time.sleep(0.5)
            while True:
                cur_pos = pyautogui.position()
                x_input = (cur_pos[0]-init_pos[0])/5 + home_pos[0]
                y_input = -(cur_pos[1]-init_pos[1])/5 + home_pos[1]
                # print(x_input,y_input)
                write_angle(inverse_kinematics((x_input,y_input)))
                time.sleep(0.05)
        elif cmd == 'flip':
            r = 25 #radius
            c_x,c_y = 0, 140 #center
            T = 1 #duration
            t = 0
            print("initializing pose...")
            enable_motor()
            write_angle(inverse_kinematics((c_x,c_y-r)))
            time.sleep(2)
            t_start = time.time()
            while t <= 20*T:
                x = c_x + sin(t*pi*2/T)*r
                y = c_y - cos(t*pi*2/T)*r
                #print(x,y)
                write_angle(inverse_kinematics((x,y)))
                t = time.time() - t_start
                time.sleep(1/setpoint_rate)
        elif cmd == 'run_circle':
            r = 50 #radius
            c_x,c_y = 0, 120 #center
            T = 0.2 #duration
            t = 0
            print("initializing pose...")
            enable_motor()
            write_angle(inverse_kinematics((c_x,c_y+r)))
            time.sleep(2)
            t_start = time.time()
            while t <= 4*T:
                x = c_x + sin(t*pi*2/T)*r
                y = c_y + cos(t*pi*2/T)*r
                #print(x,y)
                write_angle(inverse_kinematics((x,y)))
                t = time.time() - t_start
                time.sleep(1/setpoint_rate)
        elif cmd == 'run_square':
            l = 100 #square length
            c_x, c_y = 0, 120 #center
            t = 0
            T = 1 #duration
            state = 0
            x_wp = [c_x + l/2, c_x - l/2, c_x - l/2, c_x + l/2, c_x + l/2]
            y_wp = [c_y + l/2, c_y + l/2, c_y - l/2, c_y - l/2, c_y + l/2]
            print("initializing pose...")
            enable_motor()
            write_angle(inverse_kinematics((c_x+l/2,c_y+l/2)))
            time.sleep(2)
            t_start = time.time()
            while t <= 3*T:
                state = int((t / (T / 4)) % 4)
                x = (x_wp[state+1] - x_wp[state]) * (t % (T / 4)) / (T/4) + x_wp[state]
                y = (y_wp[state+1] - y_wp[state]) * (t % (T / 4)) / (T/4) + y_wp[state]
                #print(x,y)
                write_angle(inverse_kinematics((x,y)))
                t = time.time() - t_start
                time.sleep(1/setpoint_rate)
        elif cmd == 'soft':
            set_soft()
        elif cmd == 'hard':
            set_hard()
        elif cmd == 'current_pos':
            while True:
                print("---------------------------------------------")
                print(forward_kinematics(get_angle()))
                time.sleep(0.05)
        elif cmd =='fk_ik': #test forward kinematics and inverse kinematics
            while True:
                print("---------------------------------------------")
                print(inverse_kinematics(forward_kinematics(get_angle())))
                print(get_angle())
                time.sleep(0.05)
        elif cmd == 'q':
            return

if __name__ == "__main__":
    main()
