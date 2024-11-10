# Required libraries

import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import control as ct
import struct
from queue import Queue
from math import ceil
import json
import time
from scipy.signal import cont2discrete
from .motorsys import MotorSystemIoT, PATH_DATA, PATH_DEFAULT, FONT_SIZE, read_csv_file





def float2hex(value):
    val_binary = struct.pack('>f', value)
    return val_binary.hex()


def long2hex(value):
    val_binary = struct.pack('>L', value)
    return val_binary.hex()


def hex2float(str):
    value = struct.unpack('!f', bytes.fromhex(str))[0]
    return value


def hex2long(str):
    value = struct.unpack('!L', bytes.fromhex(str))[0]
    return value


def signal2hex(signal):
    hstring = ""
    for point in signal:
        hstring += float2hex(point)
    return hstring


def time2hex(time_points):
    hstring = ""
    for t in time_points:
        hstring += long2hex(t)
    return hstring


def matrix2hex(matrix):
    hstring = ""
    for row in matrix:
        for element in row:
            hstring += float2hex(element)
    return hstring

def hexframe_to_array(hexframe):
    hexlist = hexframe.split(",")
    array = [hex2float(p) for p in hexlist]
    return array


def display_immediately(fig):
    canvas = fig.canvas
    display(canvas)
    canvas._handle_message(canvas, {'type': 'send_image_mode'}, [])
    canvas._handle_message(canvas, {'type': 'refresh'}, [])
    canvas._handle_message(canvas, {'type': 'initialized'}, [])
    canvas._handle_message(canvas, {'type': 'draw'}, [])


def set_reference(system, ref_value=50):
    ref_hex = float2hex(ref_value)
    topic_pub = system.codes["USER_SYS_SET_REF"]
    message = json.dumps({"reference": ref_hex})
    system.connect()
    system.publish(topic_pub, message)
    system.disconnect()
    rcode = True
    return rcode


def set_pid(system, kp=1, ki=0.4, kd=0, N=5, beta=1, output = "angle", deadzone = 0.125):

    if output == "angle":
        type_control = 0
    elif output == "speed":
        type_control = 1
    else:
        raise ValueError("valid value for output is 'angle' or 'speed'")

    topic_pub = system.codes["USER_SYS_SET_PID"]
    kp_hex = float2hex(kp)
    ki_hex = float2hex(ki)
    kd_hex = float2hex(kd)
    N_hex = float2hex(N)
    beta_hex = float2hex(beta)
    type_control_hex = long2hex(type_control)
    deadzone_hex = float2hex(deadzone)

    message = json.dumps({"kp": kp_hex,
                          "ki": ki_hex,
                          "kd": kd_hex,
                          "N": N_hex,
                          "beta": beta_hex,
                          "typeControl": type_control_hex,
                          "deadzone": deadzone_hex})
    system.connect()
    system.publish(topic_pub, message)
    system.disconnect()
    rcode = True
    print("succesfull change of PID parameters")
    return rcode


def step_closed(system, r0=0 , r1=100, t0=0 ,  t1=1):

    def step_message(system, userdata, message):
        # This inner function is the callback of the received messages
        q.put(message)

    low_val = r0
    high_val = r1
    low_time = t0
    high_time = t1

    # reading the configuration parameters from the
    # dictionary of codes
    topic_pub = system.codes["USER_SYS_STEP_CLOSED"]
    topic_sub = system.codes["SYS_USER_SIGNALS_CLOSED"]
    sampling_time = system.codes["MOTOR_SAMPLING_TIME"]
    buffer = system.codes["BUFFER_SIZE"]

    # setting the parameters of the step response for sending to ESP32

    points_high = round(high_time / sampling_time)
    points_low = round(low_time / sampling_time)
    total_points = points_low + points_high
    frames = ceil(total_points/buffer)
    points_low_hex = long2hex(points_low)
    points_high_hex = long2hex(points_high)
    low_val_hex = float2hex(low_val)
    high_val_hex = float2hex(high_val)

    # command sent to ESP32 for obtaining the step response

    message = json.dumps({"low_val": low_val_hex,
                          "high_val": high_val_hex,
                          "points_low": points_low_hex,
                          "points_high": points_high_hex,
                          })

    # setting the callback for receiving messages
    system.client.on_message = step_message
    system.connect()
    system.subscribe(topic_sub)

    # sending the step_closed command through mqtt when config has been done
    system.publish(topic_pub, message)

    # vectors for storing the results and the experiment
    y = []
    r = []
    u = []
    t = []
    exp = []

    # Setting the graphics configuration for visualizing the experiment

    with plt.ioff():
        plt.close('all')
        fig, (ay, au) = plt.subplots(nrows=2, ncols=1, width_ratios=[1], height_ratios=[4, 1], figsize=(10, 6))
    display_immediately(fig)

    # display configuration
    fig.set_facecolor('#ffffff')
    # settings for the upper axes, depicting the model and speed data
    ay.set_title(f'Closed loop step response experiment with an initial value of  $r_0=${r0:0.2f}'
                 f' and a  final value of $r_0=${r1:0.2f}', fontsize= FONT_SIZE)
    ay.set_ylabel('Degrees/s (or Degrees)')
    ay.grid(True);
    ay.grid(color='#806600ff', linestyle='--', linewidth=0.25)
    ay.set_facecolor('#f4eed7ff')
    ay.set_xlim(0, t0 + t1  - sampling_time)
    ay.set_xlabel('Time(s)')

    #Setting the limits of figure
    py = 0.6
    delta_r = abs(r1 - r0)
    ylimits = [r0 , r1]
    ylimits = [np.min(ylimits), np.max(ylimits)]
    ay.set_ylim(ylimits[0] - py * delta_r, ylimits[1] + py * delta_r)

    au.set_facecolor('#d7f4e3ff')
    au.set_ylim(-5, 5)
    au.set_xlim(0, t0 + t1 - sampling_time )
    au.grid(color='#008066ff', linestyle='--', linewidth=0.25)
    au.set_ylabel('Volts')
    au.set_xlabel('Time(s)')

    line_r, = ay.plot(t, r, drawstyle='steps-post', color="#008066ff", linewidth=1.25) # drawstyle='steps'
    line_y, = ay.plot(t, y, color="#ff0066ff")
    line_u, = au.plot(t, u, color="#0066ffff")

    ay.legend([line_r, line_y], ['$r(t)$ (reference)', '$y(t)$ (output)'], fontsize=FONT_SIZE, loc ="lower right")
    au.legend([line_u], ['$u(t)$ (control signal)'], fontsize=FONT_SIZE, loc ="lower left")
    fig.canvas.draw()
    time.sleep(sampling_time)





    # this is the queue of messages filled by the step_message callback
    q = Queue()

    # at start we define a current frame of -1 indicating that no frame
    # has already been received
    curr_frame = -1
    sync = False
    while curr_frame < frames:
        try:
            message = q.get(True, 20* buffer * sampling_time)
        except:
            system.disconnect()
            raise TimeoutError("The connection has been lost. Please try again")
        decoded_message = str(message.payload.decode("utf-8"))
        msg_dict = json.loads(decoded_message)
        frame_hex = str(msg_dict["frame"])
        curr_frame = hex2long(frame_hex)

        if curr_frame == 1:
            sync = True
        if sync:
            rframe_hex =  str(msg_dict["r"])
            uframe_hex = str(msg_dict["u"])
            yframe_hex = str(msg_dict["y"])
            rframe = hexframe_to_array(rframe_hex)
            uframe = hexframe_to_array(uframe_hex)
            yframe = hexframe_to_array(yframe_hex)
            tframe = sampling_time * (np.arange(len(rframe)) + (curr_frame - 1) * buffer)
            r.extend(rframe)
            y.extend(yframe)
            u.extend(uframe)
            t.extend(tframe)
            line_r.set_data(t, r)
            line_y.set_data(t, y)
            line_u.set_data(t, u)
            fig.canvas.draw()
            time.sleep(sampling_time)
    ay.set_ylim([r0 - 0.05*delta_r, np.max(y) + 0.05*delta_r])
    plt.ioff()
    for ind in range(len(y)):
        exp.append([t[ind], r[ind], y[ind], u[ind]])

    np.savetxt(PATH_DATA + "DCmotor_step_closed_exp.csv", exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,r,y,u')
    np.savetxt(PATH_DEFAULT + "DCmotor_step_closed_exp.csv", exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,r,y,u')
    system.disconnect()

    return t, r, y, u


def stairs_closed(system, stairs=[90, 180, 270], duration= 1.5):
    def stairs_message(system, userdata, message):
        q.put(message)

    # reading the configuration parameters from the code's field in the plant

    topic_pub = system.codes["USER_SYS_STAIRS_CLOSED"]
    topic_sub = system.codes["SYS_USER_SIGNALS_CLOSED"]
    sampling_time = system.codes["MOTOR_SAMPLING_TIME"]
    buffer = system.codes["BUFFER_SIZE"]

    # setting the parameters of the step response for sending to ESP32

    points_stairs = len(stairs)
    points_stairs_hex = long2hex(points_stairs)
    signal_hex = signal2hex(stairs)
    duration = ceil(duration / sampling_time)
    duration_hex = long2hex(duration)

    min_val = np.min(stairs)

    max_val = np.max(stairs)
    min_val_hex = float2hex(min_val)
    max_val_hex = float2hex(max_val)

    # command sent to ESP32 for obtaining the step response
    message = json.dumps({"signal": signal_hex,
                          "duration": duration_hex,
                          "points_stairs": points_stairs_hex,
                          "min_val": min_val_hex,
                          "max_val": max_val_hex
                          })


    system.client.on_message = stairs_message

    # connecting system
    system.connect()

    # topic received from ESP32
    system.subscribe(topic_sub)

    # command sent to ESP32 for obtaining the stairs response
    system.publish(topic_pub, message)


    total_points = points_stairs * duration - 1
    frames = ceil(total_points / buffer)


    # vectors for storing the results of the experiment
    y = []
    r = []
    u = []
    t = []
    exp = []

    # Setting the graphics configuration for visualizing the experiment

    with plt.ioff():
        plt.close('all')
        fig, (ay, au) = plt.subplots(nrows=2, ncols=1, width_ratios=[1], height_ratios=[4, 1], figsize=(10, 6))
    display_immediately(fig)

    # settings for the upper axes, depicting the model and speed data
    fig.set_facecolor('#ffffff')
    ay.set_title(f'Stairs experiment with {len(stairs):d} stairs and a duration of {len(stairs)*duration:0.2f} seconds' )
    ay.set_ylabel('Degrees/s (or Degrees)')
    ay.set_xlabel('Time (seconds)')
    ay.grid(color='#806600ff', linestyle='--', linewidth=0.25)
    ay.set_facecolor('#f4eed7ff')
    line_r, = ay.plot(t, r,  color="#005544ff", linewidth=1.25)
    line_y, = ay.plot(t, y,  color="#d45500ff", linewidth=1.25)
    ay.legend([line_r, line_y], ['$r(t)$ (reference)', '$y(t)$ (output)'], fontsize=FONT_SIZE)
    ay.set_xlim(0, sampling_time * (total_points - 1))
    spany = max_val - min_val
    ay.set_ylim(np.min([0, min_val-0.1*abs(spany)]), max_val + 0.1* spany)

    au.set_facecolor('#d7f4e3ff')
    au.set_ylim(-5.5, 5.5)
    au.set_xlim(0, total_points*sampling_time)
    au.grid(color='#008066ff', linestyle='--', linewidth=0.25)
    line_u, = au.plot(t, u, color="#0066ffff")
    au.legend([line_u], ['$u(t)$ (control signal)'], fontsize=FONT_SIZE, loc ="lower right")
    au.set_xlabel('Time (s)')
    au.set_ylabel('Volts (V)')
    fig.canvas.draw()
    time.sleep(sampling_time)


    # This is the queue of messages filled by the stair_message callback
    q = Queue()

    # At beginning we define a current frame of -1 indicating that no frame
    # has already been received
    curr_frame = -1
    sync = False
    # loop for receiving dataframes from the ESP32

    while curr_frame < frames:
        try:
            # we wait for 10 seconds for a new dataframe
            message = q.get(True, 20 * buffer * sampling_time)
        except:
            # else we raise a communication error
            system.disconnect()
            raise TimeoutError("The connection has been lost. Please try again")

        # decoding the message
        decoded_message = str(message.payload.decode("utf-8"))
        msg_dict = json.loads(decoded_message)
        frame_hex = str(msg_dict["frame"])
        curr_frame = hex2long(frame_hex)
        if curr_frame == 1:
            sync = True
        if sync:
            rframe_hex = str(msg_dict["r"])
            uframe_hex = str(msg_dict["u"])
            yframe_hex = str(msg_dict["y"])
            rframe = hexframe_to_array(rframe_hex)
            uframe = hexframe_to_array(uframe_hex)
            yframe = hexframe_to_array(yframe_hex)
            tframe = sampling_time * (np.arange(len(rframe)) + (curr_frame - 1) * buffer)

            # we plot every dataframe
            r.extend(rframe)
            y.extend(yframe)
            u.extend(uframe)
            t.extend(tframe)
            line_r.set_data(t, r)
            line_y.set_data(t, y)
            line_u.set_data(t, u)
            fig.canvas.draw()
            time.sleep(sampling_time)
    #we save the results from the experiment
    for ind in range(len(y)):
        exp.append([t[ind], r[ind], y[ind], u[ind]])

    # Now, we save the results of the experiment in the provided filepath
    np.savetxt(PATH_DATA + "DCmotor_stairs_closed_exp.csv", exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,r,y,u')
    np.savetxt(PATH_DEFAULT + "DCmotor_stairs_closed_exp.csv", exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,r,y,u')
    # Now all is done, close the connection and close the figure.
    system.disconnect()
    return t, r, y, u


def set_controller(system, controller, output='angle', deadzone=0.2):
    topic_pub = system.codes["USER_SYS_SET_GENCON"]
    sampling_time = system.codes["MOTOR_SAMPLING_TIME"]
    struct = len(controller.den[0])

    if (output == "angle") & (struct == 1):
        type_control = 2
    elif (output == "speed") & (struct == 1):
        type_control = 3
    elif (output == "angle") & (struct == 2):
        type_control = 4
    elif (output == "speed") & (struct == 2):
        type_control = 5
    else:
        raise ValueError("valid value for output is 'angle' or 'speed'")

    if struct == 1:
        con = ct.tf(ct.tf(controller.num[0][0], controller.den[0][0]))
        N1, D1 = ct.tfdata(con)
        N1 = N1[0][0]
        D1 = D1[0][0]
        N1 = N1 / D1[0]
        D1 = D1 / D1[0]

        if len(N1) == len(D1):
            d1 = N1[0]
            N1 = N1 - d1* D1
            N1 = N1[1:]
        else:
            d1 = 0

        DB = np.array([-D1[1:]])
        DB = DB.T
        size = len(D1)-2
        In_1 = np.eye(size)
        ZR = np.zeros((1,size))
        Acon = np.block([[In_1], [ZR]])
        Acon = np.block([DB, Acon])
        Bcon = np.array([N1]).T
        Ccon = np.append([1], ZR)
        Dcon = np.array([d1])



    elif struct == 2:
        con1 = ct.tf(ct.tf(controller.num[0][0], controller.den[0][0]))
        con2 = ct.tf(ct.tf(controller.num[0][1], controller.den[0][1]))
        N1, D1 = ct.tfdata(con1)
        N2, D2 = ct.tfdata(con2)
        N1 = N1[0][0]
        D1 = D1[0][0]
        N1 = N1 / D1[0]
        D1 = D1 / D1[0]
        N2 = N2[0][0]
        D2 = D2[0][0]
        N2 = N2 / D2[0]
        D2 = D2/ D2[0]

        if len(N1) == len(D1):
            d1 = N1[0]
            N1 = N1 - d1* D1
            N1 = N1[1:]
        else:
            d1 = 0

        if len(N2) == len(D2):
            d2 = N2[0]
            N2 = N2 - d2* D2
            N2 = N2[1:]
        else:
            d2 = 0

        DB = np.array([-D1[1:]])
        DB = DB.T
        size = len(D1)-2
        In_1 = np.eye(size)
        ZR = np.zeros((1,size))
        Acon = np.block([[In_1], [ZR]])
        Acon = np.block([DB, Acon])
        B1 = np.array([N1]).T
        B2 = np.array([N2]).T
        Bcon = np.block([B1,B2])
        Ccon = np.append([1], ZR)
        Dcon = np.block([d1, d2])

    Ad, Bd, Cd, Dd, dt = cont2discrete((Acon, Bcon, Ccon, Dcon), sampling_time, method='bilinear')
    Cve = ct.ss(Ad, Bd, Cd, Dd)
    Ai, Bi, Ci, Di = Cve.A, Cve.B[:, 0], Cve.C, Cve.D[0][0]
    int_system = ct.ss(Ai, Bi, Ci, Di)
    int_system, T = ct.canonical_form(int_system)
    Cve = ct.similarity_transform(Cve, T)
    A = Cve.A
    B = Cve.B
    Cc = Cve.C
    Dc = Cve.D
    order = np.size(A, 0)
    In = np.diag([10000 for i in range(order)])
    L, S, E = ct.dlqr(np.transpose(A), np.transpose(Cc), In, 1)
    L = np.transpose(L)
    Ac = A - L * Cc
    Bc = B - L * Dc
    if struct == 1:
        Bc = np.block([Bc, -Bc])
        Dc = np.array([[Dc[0][0], -Dc[0][0]]])
    A_hex = matrix2hex(Ac)
    B_hex = matrix2hex(Bc)
    C_hex = matrix2hex(Cc)
    D_hex = matrix2hex(Dc)
    L_hex = matrix2hex(L)
    order_hex = long2hex(order)
    deadzone_hex = float2hex(deadzone)
    type_control_hex = long2hex(type_control)
    message = json.dumps({"order": order_hex,
                          "A": A_hex,
                          "B": B_hex,
                          "C": C_hex,
                          "D": D_hex,
                          "L": L_hex,
                          "typeControl": type_control_hex,
                          "deadzone": deadzone_hex
                          })

    system.connect()
    system.publish(topic_pub, message)
    system.disconnect()
    print("Controller has been succesfully uploaded to UNDCMotor")
    return



def profile_closed(system, timevalues = [0, 1, 2 ,3], refvalues = [0, 360, 360, 0]):
    def profile_message(system, userdata, message):
        # This is the callback for receiving messages from the plant
        q.put(message)

    # reading the configuration parameters from the code's field in the plant

    topic_pub = system.codes["USER_SYS_PROFILE_CLOSED"]
    topic_sub = system.codes["SYS_USER_SIGNALS_CLOSED"]
    sampling_time = system.codes["MOTOR_SAMPLING_TIME"]
    buffer = system.codes["BUFFER_SIZE"]

    # setting the parameters of the step response for sending to ESP32

    int_timevalues = [round(p/sampling_time) for p in timevalues]
    if int_timevalues[0] != 0:
        int_timevalues.insert(0, int_timevalues[0]-1)
        int_timevalues.insert(0,0)
        refvalues.insert(0,0)
        refvalues.insert(0,0)


    int_timevalues_hex = time2hex(int_timevalues)
    refvalues_hex = signal2hex(refvalues)
    points = len(int_timevalues)
    points_hex = long2hex(points)

    # user's command for obtaining the profile
    # al values are transmitted in hexadecimal
    min_val = np.min(refvalues)
    max_val = np.max(refvalues)
    min_val_hex = float2hex(min_val)
    max_val_hex = float2hex(max_val)

    message = json.dumps({"timevalues":  int_timevalues_hex,
                          "refvalues":   refvalues_hex,
                          "points":      points_hex,
                          "min_val":     min_val_hex,
                          "max_val":     max_val_hex,
                          })

    # setting the callback fro receiving data from the ESP32 for obtaining the profile response
    system.client.on_message = profile_message

    # connecting the system
    system.connect()

    # subscribing to topic published by ESP32
    system.subscribe(topic_sub)

    # command sent to ESP32 for obtaining the profile response
    system.publish(topic_pub, message)

    # setting the total of points and the total of frames
    total_points = int_timevalues[-1] + 1
    frames = ceil(total_points / buffer)

    # vectors for storing the results of the experiment
    y = []
    r = []
    u = []
    t = []
    exp = []

    # Setting the graphics configuration for visualizing the experiment

    with plt.ioff():
        plt.close('all')
        fig, (ay, au) = plt.subplots(nrows=2, ncols=1, width_ratios=[1], height_ratios=[4, 1], figsize=(10, 6))
    display_immediately(fig)

    # settings for the upper axes, depicting the model and speed data
    ay.set_title(f'Profile response experiment with a duration of {timevalues[-1]:0.2f} seconds and {len(timevalues):d} edges')
    ay.set_ylabel('Degrees/s (or Degrees)')
    ay.set_xlabel('Time (seconds)')
    ay.grid(color='#806600ff', linestyle='--', linewidth=0.25)
    ay.set_facecolor('#f4eed7ff')
    line_r, = ay.plot(t, r,  color="#005544ff", linewidth=1.25)
    line_y, = ay.plot(t, y,  color="#d45500ff", linewidth=1.25)
    ay.legend([line_r, line_y], ['$r(t)$ (reference)', '$y(t)$ (output)'], fontsize=FONT_SIZE)
    ay.set_xlim(0, sampling_time * (total_points - 1))
    spany = max_val - min_val
    ay.set_ylim( min_val-0.1*abs(spany), max_val + 0.1* spany)

    au.set_facecolor('#d7f4e3ff')
    au.set_ylim(-5.5, 5.5)
    au.set_xlim(0, total_points*sampling_time)
    au.set_xlabel('Time (s)')
    au.set_xlabel('Volts (V)')
    au.grid(color='#008066ff', linestyle='--', linewidth=0.25)
    line_u, = au.plot(t, u, color="#0066ffff")
    au.legend([line_u], ['$u(t)$ (Volts)'], fontsize=FONT_SIZE)

    line_r.set_data(t, r)
    line_y.set_data(t, y)
    line_u.set_data(t, u)
    fig.canvas.draw()
    time.sleep(sampling_time)


    # This is the queue of messages filled by the stair_message callback
    q = Queue()

    # At beginning we define a current frame of -1 indicating that no frame
    # has already been received
    curr_frame = -1
    sync = False
    # loop for receiving dataframes from the ESP32
    while curr_frame < frames:
        try:
            # we wait for 10 seconds for a new dataframe
            message = q.get(True, 20 * buffer * sampling_time)
        except:
            # else we raise a communication error
            system.disconnect()
            raise TimeoutError("The connection has been lost. Please try again")

        # decoding the message
        decoded_message = str(message.payload.decode("utf-8"))
        msg_dict = json.loads(decoded_message)
        frame_hex = str(msg_dict["frame"])
        curr_frame = hex2long(frame_hex)

        if curr_frame == 1:
            sync = True
        if sync:
            rframe_hex = str(msg_dict["r"])
            uframe_hex = str(msg_dict["u"])
            yframe_hex = str(msg_dict["y"])
            rframe = hexframe_to_array(rframe_hex)
            uframe = hexframe_to_array(uframe_hex)
            yframe = hexframe_to_array(yframe_hex)
            tframe = sampling_time * (np.arange(len(rframe)) + (curr_frame - 1) * buffer)

            # we plot every dataframe
            r.extend(rframe)
            y.extend(yframe)
            u.extend(uframe)
            t.extend(tframe)
            line_r.set_data(t, r)
            line_y.set_data(t, y)
            line_u.set_data(t, u)
            fig.canvas.draw()
            time.sleep(sampling_time)

    # we save the results from the experiment
    for ind in range(len(y)):
        exp.append([t[ind], r[ind], y[ind], u[ind]])

    # Now, we save the results of the experiment in the provided filepath
    np.savetxt(PATH_DATA + "DCmotor_profile_closed_exp.csv" , exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,r,y,u')
    np.savetxt(PATH_DEFAULT + "DCmotor_profile_closed_exp.csv" , exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,r,y,u')
    # Now all is done, close the connection and close the figure.
    system.disconnect()
    return t, y, r, u

def read_model_pbrs():
    with open(PATH + 'DCmotor_fo_model_pbrs.csv', newline='') as file:
        reader = csv.reader(file)
        # Iterate over each row in the CSV file
        num_line = 0
        for row in reader:
            if num_line != 0:
               alpha = float(row[0])
               tau = float(row[1])
            num_line += 1
        return alpha, tau


if __name__ == "__main__":
    motor1 = MotorSystemIoT(plant_number=2201)
    print(motor1.codes["SYS_USER_SIGNALS_CLOSED"])
    set_pid(motor1, kp=0.042648109526048, ki=0.02, kd=0, N=11.9, beta=1, output = "angle", deadzone=0.2)
    st = [50,80,-50,100]
    import control as ct
    s = ct.TransferFunction.s
    #C= 0.042 + 0.1/s

    #set_controller(motor1, C)
    profile_closed(motor1)

