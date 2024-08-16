# Required libraries

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint
from scipy.optimize import minimize
from scipy.interpolate import interp1d, PchipInterpolator

from scipy.stats import linregress
import control as ct
from .motorsys import MotorSystemIoT, PATH_DATA, PATH_DEFAULT, FONT_SIZE
from .controlsys import  long2hex, float2hex, hex2long, set_pid, hex2float, hexframe_to_array, display_immediately
import json
from math import ceil
from queue import Queue
from pathlib import Path
import csv
from pathlib import Path
import time





PBRS_LENGTH = 1023

def read_csv_file3(filepath):
    with open(filepath, newline='') as file:
        reader = csv.reader(file)
        # Iterate over each row in the CSV file
        num_line = 0
        t = []
        u = []
        y = []
        for row in reader:
            if num_line != 0:
               t.append(float(row[0]))
               u.append(float(row[1]))
               y.append(float(row[2]))
            num_line += 1
        return t, u, y



def step_open(system, u0=1.5, u1=3.5, t0=1, t1=1):
    def step_message(system, userdata, message):
        q.put(message)
    # reading the configuration parameters from the
    # dictionary of codes
    low_val = u0
    high_val = u1
    low_time = t0
    high_time = t1
    topic_pub = system.codes["USER_SYS_STEP_OPEN"]
    topic_sub = system.codes["SYS_USER_SIGNALS_OPEN"]
    sampling_time = system.codes["MOTOR_SAMPLING_TIME"]
    buffer = system.codes["BUFFER_SIZE"]

    # setting the parameters of the step response for sending to ESP32

    points_high = round(high_time / sampling_time) + 1
    points_low = round(low_time / sampling_time)
    total_points = points_low + points_high
    frames = ceil(total_points / buffer)
    points_low_hex = long2hex(points_low)
    points_high_hex = long2hex(points_high)
    low_val_hex = float2hex(low_val)
    high_val_hex = float2hex(high_val)

    # command sent to ESP32 for obtaining the  open loop step response
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
    u = []
    t = []
    exp = []
    # Setting the graphics configuration for visualizing the experiment

    # display config
    with plt.ioff():
        plt.close('all')
        fig, (yax, uax) = plt.subplots(nrows=2, ncols=1, width_ratios=[1], height_ratios=[2, 1], figsize=(10, 6))
    display_immediately(fig)

    yax.set_title(f'Experiment of Step response with a duration of {total_points * sampling_time: 0.2f} seconds', fontsize=FONT_SIZE)
    yax.set_ylabel('Degrees/s')
    yax.set_xlabel('Time (s)')
    yax.grid(True);
    yax.grid(color='#1a1a1a40', linestyle='--', linewidth=0.25)
    yax.set_facecolor('#f4eed7')


    uax.set_xlabel('Time (s)')
    uax.set_ylabel('Volts')
    uax.grid(True);
    uax.set_facecolor('#d7f4ee')
    uax.grid(color='#1a1a1a40', linestyle='--', linewidth=0.25)


    line_u, = uax.plot(t, u, drawstyle='steps-post', color="#338000")
    line_y, = yax.plot(t, y,  color="#d40055")
    uax.legend([line_u], ['$u(t)$ (open loop voltage)'], fontsize=FONT_SIZE, loc="lower right")
    yax.legend([line_y], ['$y(t)$ (speed in Deg/s)'], fontsize=FONT_SIZE, loc="lower right")
    #Setting the limits of figure
    pu = 0.1
    ulimits = [low_val, high_val]
    ylimits = system.speed_from_volts(ulimits)
    ylimits = [np.min(ylimits)-50, np.max(ylimits)+50]
    ulimits = [np.min(ulimits), np.max(ulimits)]
    delta_u = ulimits[1] - ulimits[0]
    uax.set_ylim(ulimits[0] - pu*delta_u, ulimits[1] + pu*delta_u)
    uax.set_xlim(0, sampling_time * (total_points - 1))
    yax.set_xlim(0, sampling_time * (total_points - 1))
    yax.set_ylim(ylimits[0] - 25, ylimits[1] + 50)
    # fig.canvas.draw()
    # time.sleep(sampling_time)

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
            uframe_hex = str(msg_dict["u"])
            yframe_hex = str(msg_dict["y"])
            uframe = hexframe_to_array(uframe_hex)
            yframe = hexframe_to_array(yframe_hex)
            tframe = sampling_time * (np.arange(len(yframe)) + (curr_frame - 1) * buffer)
            y.extend(yframe)
            u.extend(uframe)
            t.extend(tframe)
            line_u.set_data(t, u)
            line_y.set_data(t, y)
            fig.canvas.draw()
            time.sleep(sampling_time)

    for ind in range(len(y)):
        exp.append([t[ind], u[ind], y[ind]])

    np.savetxt(PATH_DATA + "DCmotor_step_open_exp.csv", exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,u,y')
    np.savetxt(PATH_DEFAULT + "DCmotor_step_open_exp.csv", exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,u,y')
    system.disconnect()
    return t, u, y




def prbs_open(system, low_val = 2, high_val = 4, divider = 2):
    def pbrs_message(system, userdata, message):
        q.put(message)

    # This is the command code for PBRS
    topic_pub = system.codes["USER_SYS_PRBS_OPEN"]
    # This is the topic for receiving data
    topic_sub = system.codes["SYS_USER_SIGNALS_OPEN"]
    # reading the size of the buffer
    buffer = system.codes["BUFFER_SIZE"]
    # sampling time
    sampling_time = system.codes["MOTOR_SAMPLING_TIME"]


    # calculating the length of the pbrs which will be applied to the DC motor

    total_points = PBRS_LENGTH * divider

    # setting the parameters of the step response for sending to ESP32
    frames = ceil(total_points/buffer)
    low_val_hex = float2hex(low_val)
    high_val_hex = float2hex(high_val)
    divider_hex = long2hex(divider)

    # command sent to ESP32 for obtaining the step response
    message = json.dumps({"low_val": low_val_hex,
                          "high_val": high_val_hex,
                          "divider": divider_hex
                          })

    # setting the callback for receiving messages
    system.client.on_message = pbrs_message
    # connecting to IoT DC motor
    system.connect()
    # Subscribing to the topic through which the IoT plant transmits.
    system.subscribe(topic_sub)

    # sending the step_closed command through mqtt when config has been done
    system.publish(topic_pub, message)

    # vectors for storing the results and the experiment
    y = []
    u = []
    t = []
    exp = []

    # Setting the graphics configuration for visualizing the experiment
    print("Starting experiment, please wait...")
    with plt.ioff():
        plt.close('all')
        fig, (yax, uax) = plt.subplots(nrows=2, ncols=1, width_ratios=[1], height_ratios=[3, 1], figsize=(10, 6))
    display_immediately(fig)


    fig.set_facecolor('#ffffff')

    fig.suptitle( f'PRBS identification with {total_points:d} samples'
                  f' and a duration of {total_points * sampling_time: 0.2f} seconds', fontsize=10)

    yax.set_title("Starting experiment, please wait...")
    yax.set_ylabel('Speed (Degrees/s)')
    yax.grid(True);
    yax.grid(color='#1a1a1a40', linestyle='--', linewidth=0.25)
    yax.set_facecolor('#f4eed7')


    uax.set_xlabel('Time (s)')
    uax.set_ylabel('Input (V)')
    uax.grid(True);
    uax.set_facecolor('#d7f4ee')
    uax.grid(color='#1a1a1a40', linestyle='--', linewidth=0.25)
    line_u, = uax.plot(t, u, drawstyle='steps-post', color="#338000")
    line_y, = yax.plot(t, y,  drawstyle='steps-post',  color="#d40055")

    #Setting the limits of figure
    pu = 0.1
    ulimits = [low_val, high_val]
    ylimits = system.speed_from_volts(ulimits)
    ylimits = [np.min(ylimits), np.max(ylimits)]
    ulimits = [np.min(ulimits), np.max(ulimits)]
    delta_u = ulimits[1] - ulimits[0]
    uax.set_ylim(ulimits[0] - pu*delta_u, ulimits[1] + pu*delta_u)
    uax.set_xlim(0, sampling_time * (total_points - 1))
    yax.set_xlim(0, sampling_time * (total_points - 1))
    yax.set_ylim(ylimits[0] - 25, ylimits[1] + 25)
    fig.canvas.draw()
    # this is the queue of messages filled by the step_message callback
    q = Queue()

    # at start we define a current frame of -1 indicating that no frame
    # has already been received
    curr_frame = -1
    sync = False
    # we keep the loop until all frames have been receuves
    while curr_frame < frames:
        try:
            # We try to extract the last payload from the queue, and if it is empty
            # we wait for at least 10 seconds
            message = q.get(True, 20 * buffer * sampling_time)
        except:
            # after ten seconds we raise a timeout
            system.disconnect()
            raise TimeoutError("The connection has been lost. Please try again")

        # this es the current received  message
        decoded_message = str(message.payload.decode("utf-8"))
        msg_dict = json.loads(decoded_message)
        # reading the current frame's number
        frame_hex = str(msg_dict["frame"])
        curr_frame = hex2long(frame_hex)
        if curr_frame == 1:
            sync = True
        if sync:

            # reading the current frame values for the input u and the output y
            uframe_hex = str(msg_dict["u"])
            yframe_hex = str(msg_dict["y"])
            uframe = hexframe_to_array(uframe_hex)
            yframe = hexframe_to_array(yframe_hex)

            # Defining the current frame of time
            tframe = sampling_time * (np.arange(len(uframe)) + (curr_frame - 1) * buffer)

            # updating t,u, and y vectors
            y.extend(yframe)
            u.extend(uframe)
            t.extend(tframe)

            #we plot 5 frames at start
            if curr_frame <= 5:
                yax.set_title("")
                uax.set_xlim(0, 5*buffer*sampling_time)
                yax.set_xlim(0, 5*buffer*sampling_time)


            # after frame 5 we plot the last 6 frames
            elif curr_frame >=6 :
                uax.set_xlim(t[-6*buffer], t[-1])
                yax.set_xlim(t[-6*buffer], t[-1])


            # plotting the current data

            line_u.set_data(t, u)
            line_y.set_data(t, y)
            fig.canvas.draw()
            time.sleep(sampling_time)

    # preparing the matrix for storing the results of the identification experiment
    for ind in range(len(y)):
        exp.append([t[ind],  u[ind], y[ind]])

    # Saving the results of the experiment, stored in list 'exp', in the given filepath
    np.savetxt(PATH_DATA + 'DCmotor_prbs_open_exp.csv', exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,u,y')
    np.savetxt(PATH_DEFAULT + 'DCmotor_prbs_open_exp.csv', exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,u,y')
    system.disconnect()
    return t, u, y
#
#

def step_open_staticgain(system, low_val=1.5, high_val=3.5, low_time=1, high_time=1):
    def step_message(system, userdata, message):
        q.put(message)
    # reading the configuration parameters from the
    # dictionary of codes

    topic_pub = system.codes["USER_SYS_STEP_OPEN"]
    topic_sub = system.codes["SYS_USER_SIGNALS_OPEN"]
    sampling_time = system.codes["MOTOR_SAMPLING_TIME"]
    buffer = system.codes["BUFFER_SIZE"]


    # setting the parameters of the step response for sending to ESP32

    points_high = round(high_time / sampling_time) + 1
    points_low = round(low_time / sampling_time)
    total_points = points_low + points_high
    frames = ceil(total_points / buffer)
    points_low_hex = long2hex(points_low)
    points_high_hex = long2hex(points_high)
    low_val_hex = float2hex(low_val)
    high_val_hex = float2hex(high_val)

    # command sent to ESP32 for obtaining the  open loop step response
    message = json.dumps({"low_val": low_val_hex,
                          "high_val": high_val_hex,
                          "points_low": points_low_hex,
                          "points_high": points_high_hex,
                          })

    # setting the callback for receiving messages
    system.client.on_message = step_message

    if not system.client.is_connected():
        system.connect()
        system.subscribe(topic_sub)
        # sending the step_closed command through mqtt when config has been done

    system.publish(topic_pub, message)
    # vectors for storing the results and the experiment
    y = []
    u = []
    t = []


    # this is the queue of messages filled by the step_message callback
    q = Queue()

    with plt.ioff():
        fig = plt.gcf()
        ax = fig.get_axes()[0]

    line_y, = ax.plot(t, y, linestyle = 'solid', color="#d3bc5fff", linewidth=1)


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
            uframe_hex = str(msg_dict["u"])
            yframe_hex = str(msg_dict["y"])
            uframe = hexframe_to_array(uframe_hex)
            yframe = hexframe_to_array(yframe_hex)
            tframe = sampling_time * (np.arange(len(yframe)) + (curr_frame - 1) * buffer)
            y.extend(yframe)
            u.extend(uframe)
            t.extend(tframe)
            line_y.set_data(t, y)
            fig.canvas.draw()
            time.sleep(sampling_time)
    line_y.set_data([], [])
    return u, y





def get_static_model(system):

    # This is the configuration for the figure displayed while acquiring data
    yee = []
    uee = []

    with plt.ioff():
        plt.close('all')
        fig, ax = plt.subplots(figsize=(10, 6))
    display_immediately(fig)

    ax.set_title('Static gain response experiment for UNDCMotor')
    ax.set_xlabel('Input (Volts)')
    ax.set_ylabel('Steady state speed (Degrees/s)')

    ax.grid(True);
    ax.set_facecolor('#f4eed7')
    ax.set_yticks(np.arange(-700, 800, 100))
    ax.set_xticks([-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5])
    ax.grid(color='#1a1a1a40', linestyle='--', linewidth=0.25)
    line_exp, = ax.plot(uee, yee, color="#00aa00", linewidth=1.5, marker=r"$\circ$", markeredgewidth=0.1)
    ax.set_xlim(-5, 5)
    ax.set_ylim(-780, 780)
    box = dict(boxstyle='round,pad=0.5', facecolor='white', edgecolor='white', alpha=0.5)
    fig.canvas.draw()



    # These are the parameters for obtaining the actuator response
    timestep = 3
    points = 20
    dz_point = 0.3

    # # This is the set of step responses for obtaining steady state in speed
    delta_dz = 0.02
    u_pos = np.logspace(np.log10(dz_point - delta_dz), np.log10(5), points , endpoint=True, base=10)
    u_neg = -u_pos[::-1]
    u_tot = np.concatenate((u_neg, u_pos))
    exp = []
    for ui in u_tot:
        try:
            u, y = step_open_staticgain(system, 0, ui,0, timestep)
        except:
            time.sleep(4)
            u, y = step_open_staticgain(system, 0, ui, 0, timestep)



        yf = np.mean(y[-50:])
        uf = u[-1]
        exp.append([uf, yf])
        yee.append(yf)
        uee.append(ui)
        line_exp.set_data(uee, yee)
        ax.legend([line_exp], [f'Last input voltage: {uf:0.2f} V\nLast steady state speed: {yf:0.2f} deg/s'], fontsize=FONT_SIZE, loc='upper left')
        fig.canvas.draw()
        time.sleep(0.5)

    np.savetxt(PATH_DEFAULT + "DCmotor_static_gain_response.csv",
               exp, delimiter=",", fmt="%0.8f", comments="", header='u,y')
    np.savetxt(PATH_DATA + "DCmotor_static_gain_response.csv",
               exp, delimiter=",", fmt="%0.8f", comments="", header='u,y')
    system.disconnect()
    print("Static model esperiment has been completed")
    return uee, yee


def get_fomodel_step(system, yop=400, usefile=False):
    """This function allows to obtain the first order model
    from the step response"""

    ymax = system.speed_from_volts(5)
    ymin = system.speed_from_volts(-5)
    sigma = 50  # this is the deviation from the operation point
    timestep = 3
    if ymin <= yop <= ymin + sigma:
        ua = system.volts_from_speed(-5)
        ub = system.volts_from_speed(yop + sigma)
    elif ymin + sigma < yop <= -200:
        ua = system.volts_from_speed(yop - sigma)
        ub = system.volts_from_speed(yop + sigma)
        timestep = 4
    elif -200 < yop < 0:
        ua = system.volts_from_speed(-200)
        ub = system.volts_from_speed(-100)
        timestep = 5
    elif 0 <= yop < 200:
        ua = system.volts_from_speed(100)
        ub = system.volts_from_speed(200)
        timestep = 5
    elif 200 <= yop < ymax-sigma:
        ua = system.volts_from_speed(yop - sigma)
        ub = system.volts_from_speed(yop + sigma)
    elif ymax - sigma <= yop <= ymax:
        ua = system.volts_from_speed(yop - sigma)
        ub = 5
    else:
        raise ValueError(f"The maximum speed for this motor is {ymax:.2f} \n\t\t\t and the minimum is {ymin:.2f} ")


    if  not usefile:
        try:
            t, u, y = step_open(system, ua, ub, timestep, timestep)
        except:
            system.disconnect()
            raise TimeoutError("The connection has been lost. Please try again")
    # we get the step response near to operation point
    t, u, y = read_csv_file3(PATH_DATA + 'DCmotor_step_open_exp.csv')

    # we interpolate the experimental response
    interp = PchipInterpolator(t, y)

    # we estimate the steady state speeds achieved during the initial value of step.
    ta = [t0 for t0 in t if t0 > timestep - 1 and t0 < timestep]
    ya = np.mean(interp(ta))

    # we estimate the steady state speeds achieved during the final value of the step.
    tb = [t0 for t0 in t if t0 > 2*timestep - 1 and t0 < 2 * timestep]
    yb = np.mean(interp(tb))

    # we use the mean value theorem to approximate the gain of the motor
    delta_y = yb - ya
    delta_u = ub - ua
    alpha = delta_y / delta_u

    #  We use four point of the the step response

    y_t1  = ya + 0.2 * delta_y
    y_t2  = ya + 0.4 * delta_y
    y_t3  = ya + 0.63212 * delta_y
    y_t4  = ya + 0.8 * delta_y

    # with this value, we can approximate the value of tau
    # solving the inverse equation by means of the interpolator
    roots_t1 = interp.solve(y_t1, extrapolate=False)
    roots_t2 = interp.solve(y_t2, extrapolate=False)
    roots_t3 = interp.solve(y_t3, extrapolate=False)
    roots_t4 = interp.solve(y_t4, extrapolate=False)

    # We take the maximum value of the roots in the event that the noise produces multiple values.
    # We also need to substract the time in which the step changes

    t1  = np.max(roots_t1) - timestep
    t2  =  np.max(roots_t2) - timestep
    tau3 = np.mean(roots_t3) - timestep
    t4 =  np.min(roots_t4) - timestep

    # We obtain 4 estimates of tau in 4 different points
    tau1 = t1 / 0.2231
    tau2 = t2 / 0.5108
    tau4 = t4 /1.6094

    # we average the 4 estimated values for obtainig tau
    tau = (tau1 + tau2 + tau3 + tau4)/4

    # we build the model
    G = ct.tf(alpha, [tau, 1])

    # we calculate the step response from the model
    um = np.array(u) - u[0]  # it is required to compute the LTI model with a signal starting in 0.
    tm, ym = ct.forced_response(G, t, um)

    # we add the initial speed to compare
    ym = ym + ya

    if usefile:
        with plt.ioff():
            plt.close("all")
            fig, (ay, au) = plt.subplots(nrows=2, ncols=1, width_ratios=[1], height_ratios=[4, 1], figsize=(10, 6))
        display_immediately(fig)
        fig.set_facecolor('#ffffff')

    else:
        with plt.ioff():
            fig = plt.gcf()
            ay, au = fig.get_axes()
            ay.cla()
            au.cla()


    # settings for the upper axes, depicting the model and speed data
    ay.set_title('Estimated first order model for UNDCMotor')
    ay.set_ylabel('Speed (Degrees/s)')
    ay.grid(True);
    ay.grid(color='#1a1a1a40', linestyle='--', linewidth=0.25)
    ay.set_facecolor('#f4eed7')
    ay.set_xlim(0, 2*timestep)
    box = dict(boxstyle='round,pad=0.5', facecolor='white', edgecolor='white', alpha=0.5)
    ay.text(timestep + tau + 1, (ya+yb)/2, r'$\Delta_{y,e}=%0.3f$'%delta_y, fontsize=FONT_SIZE, color='#ff0066',
             ha='center', va='bottom', bbox=box)
    ay.text(timestep + tau + 0.2, ya + 0.63212*delta_y,  r'$\tau = %0.3f$'%tau, fontsize=FONT_SIZE, color='#ff0066')
    ay.set_ylabel('Speed (Deg/s)')
    # settings for the lower, depicting the input
    au.set_xlim(0, 2 *timestep)
    au.grid(True);
    au.set_facecolor('#d7f4ee')
    au.grid(color='#1a1a1a40', linestyle='--', linewidth=0.25)
    au.text(timestep + 1, (ua+ub)/2, r'$\Delta_u=%0.2f$'%delta_u, fontsize=FONT_SIZE, color="#00aa00",
             ha='center', va='bottom', bbox=box)
    au.set_xlabel('Time (s)')
    au.set_ylabel('Volts (V)')


    line_exp, = ay.plot(t, y, color="#0088aa", linewidth=1.5, linestyle=(0, (1, 1)))
    line_mod, = ay.plot(tm, ym, color="#ff0066", linewidth=1.5, )
    ay.plot(timestep + tau, ya + 0.63212*delta_y , color="#ff0066", linewidth=1.5, marker=".", markersize=13)
    line_u, = au.plot(t, u, color="#00aa00")
    modelstr = r"Model $G(s)= \frac{\alpha_m}{\tau_m\,s + 1} = \frac{%0.3f }{%0.3f\,s+1}$" %(alpha, tau)

    ay.legend([line_exp, line_mod], ['Data', modelstr], fontsize=FONT_SIZE)
    au.legend([line_u], ['Input'], fontsize = FONT_SIZE)

    exp = [[alpha, tau]]
    np.savetxt(PATH_DATA + "DCmotor_fomodel_step.csv", exp, delimiter=",", fmt="%0.8f", comments="", header='alpha, tau')
    np.savetxt(PATH_DEFAULT + "DCmotor_fomodel_step.csv", exp, delimiter=",", fmt="%0.8f", comments="",
               header='alpha, tau')
    system.disconnect()
    return G


def get_models_prbs(system, yop = 400, usefile = False):

    norm = np.linalg.norm

    def simulate_fomodel(x):
        # this function simulates the model
        alpha, tau = x
        s = ct.TransferFunction.s
        G = alpha / (tau*s + 1)
        tsim, ysim = ct.forced_response(G, t, um)
        return G, ysim

    def simulate_somodel(x):
        # this function simulates the model
        alpha, tau1, tau2 = x
        s = ct.TransferFunction.s
        G = alpha / (( tau1*s + 1)*(tau2*s + 1))
        tsim, ysim = ct.forced_response(G, t, um)
        return G, ysim

    def objective_fo(x):
        # simulate model
        G, ysim = simulate_fomodel(x)
        # return objective
        return norm(ysim - ym)

    def objective_so(x):
        # simulate model
        G, ysim = simulate_somodel(x)
        # return objective
        return norm(ysim - ym)


    ymax = system.speed_from_volts(5)
    ymin = system.speed_from_volts(-5)
    sigma = 50  # this is the deviation from the operation point

    if ymin <= yop <= ymin + sigma:
        ua = system.volts_from_speed(-5)
        ub = system.volts_from_speed(yop + sigma)
    elif ymin + sigma < yop <= -200:
        ua = system.volts_from_speed(yop - sigma)
        ub = system.volts_from_speed(yop + sigma)
    elif -200 < yop < 0:
        ua = system.volts_from_speed(-200)
        ub = system.volts_from_speed(-100)
    elif 0 <= yop < 200:
        ua = system.volts_from_speed(200)
        ub = system.volts_from_speed(100)
    elif 200 <= yop < ymax - sigma:
        ua = system.volts_from_speed(yop + sigma)
        ub = system.volts_from_speed(yop - sigma)
    elif ymax - sigma <= yop <= ymax:
        ua = system.volts_from_speed(yop - sigma)
        ub = 5
    else:
        raise ValueError(f"The maximum speed for this motor is {ymax:.2f} \n\t\t\t and the minimum is {ymin:.2f} ")


    if not usefile:
        try:
            prbs_open(system, low_val=ua, high_val=ub, divider=4)
        except:
            system.disconnect()
            raise TimeoutError("The connection has been lost. Please try again")

    t, u, y = read_csv_file3(PATH_DATA + 'DCmotor_prbs_open_exp.csv')
    ya = system.speed_from_volts(ua)
    yb = system.speed_from_volts(ub)
    delta_y = yb - ya
    delta_u = ub - ua

    ymean = np.mean(y)
    um = np.array(u) -  np.mean(u)
    ym = np.array(y) - ymean
    """Now we obtain the initial parameters for the model
                           alpha
             G(s) =  ----------------------
                     (tau1*s +1)(tau2*s+1)
    """
    alpha_0 = delta_y / delta_u
    tau1_0 = 0.25
    tau2_0 = tau1_0/10
    x02 = [alpha_0, tau1_0, tau2_0]
    x01 = [alpha_0, tau1_0]

    # These are the bounds for alpha, tau1 and tau2

    bounds2 = [(0, 2500), (0 , 1), (0, 0.5)]
    bounds1 = bounds2[0:2]

    # Now we run the optimization algorithm
    print(f'Starting optimization for first order model...\n\t Initial cost function: {objective_fo(x01):.2f}' )
    solution = minimize(objective_fo, x0=x01, bounds= bounds1)
    xmin = solution.x
    alpha, tau = xmin
    fmin = objective_fo(xmin)
    print(f'\t Final cost function: {fmin:.2f}' )
    print(f'alpha={alpha:.2f} \t tau1={tau:.3f}')

    # We compare the experimental data with the simulation model
    G1, ysim1 = simulate_fomodel(xmin)
    r1 = 100*(1 - norm(ym - ysim1) / norm(ym))

    print(f'\n\nStarting optimization for second order model...\n\t Initial cost function: {objective_so(x02):.2f}' )
    solution = minimize(objective_so, x0=x02, bounds= bounds2)
    xmin = solution.x
    alpha2, tau1, tau2 = xmin
    fmin = objective_so(xmin)
    print(f'\t Final cost function: {fmin:.2f}' )
    print(f'alpha={alpha2:.2f} \t tau1={tau1:.3f} \t tau2={tau2:.3f}')

    # We compare the experimental data with the simulation model
    G2, ysim2 = simulate_somodel(xmin)
    r2 = 100*(1 - norm(ym - ysim2) / norm(y - np.mean(y)))

    # we calculate the step response from the model
    # now we compare the model with the experimental data
    if usefile:
        with plt.ioff():
            plt.close("all")
            fig, (ay, au) = plt.subplots(nrows=2, ncols=1, width_ratios=[1], height_ratios=[4, 1], figsize=(10, 6))
            fig.set_facecolor('#ffffff')
        display_immediately(fig)


    else:
        with plt.ioff():
            fig = plt.gcf()
            ay, au = fig.get_axes()
            ay.cla()
            au.cla()

    # settings for the upper axes, depicting the model and speed data
    ay.set_ylabel('Speed (Degrees/s)')
    ay.grid(True)
    ay.grid(color='#1a1a1a40', linestyle='--', linewidth=0.125)
    ay.set_facecolor('#f4eed7')
    xlimits = [t[-1]-20, t[-1]]
    ay.set_xlim(xlimits[0], xlimits[1])
    ay.set_ylabel('Speed (Degrees/s)')

    # settings for the lower, depicting the input
    au.set_xlim(xlimits[0], xlimits[1])
    au.set_facecolor('#d7f4ee')
    au.grid(color='#1a1a1a40', linestyle='--', linewidth=0.125)

    au.set_xlabel('Time (seconds)')
    au.set_ylabel('Voltage (V)')

    line_exp, = ay.plot(t, ym + ymean, color="#0088aaAF", linewidth=1.5, linestyle=(0, (1, 1)))
    line_model1, = ay.plot(t, ysim1 + ymean, color="#00AA44", linewidth=1.5, )
    line_model2, = ay.plot(t, ysim2 + ymean, color="#ff0066", linewidth=1.5, )


    line_u, = au.plot(t, u, color="#00aa00")
    modelstr1 = r"Model $G_1(s) = \frac{%0.3f }{%0.3f\,s+1}$ ($FIT = %0.1f$" % (alpha, tau, r1) + "%)"
    modelstr2 = r"Model $G_2(s) = \frac{%0.3f }{(%0.3f\,s+1)(%0.3f\,s+1)}$ ($FIT = %0.1f$"%(alpha2, tau1, tau2, r2) + "%)"
    ay.set_title("Comparison of first and second order models estimated with a PRBS signal at the point $y_{OP}=%0.1f^o/s$."%yop)
    ay.legend([line_exp, line_model1, line_model2], ['Data', modelstr1, modelstr2],
              fontsize=FONT_SIZE, loc = 'lower right',framealpha=0.95)
    au.legend([line_u], ['PRBS Input'], fontsize=FONT_SIZE)
    fig.canvas.draw()
    fo_model = [[alpha, tau]]
    so_model = [[alpha2, tau1, tau2]]
    np.savetxt(PATH_DATA + "DCmotor_fo_model_pbrs.csv", fo_model, delimiter=",",
               fmt="%0.8f", comments="", header='alpha, tau')

    np.savetxt(PATH_DEFAULT + "DCmotor_so_model_pbrs.csv", so_model, delimiter=",",
               fmt="%0.8f", comments="", header='alpha2, tau1, tau2')

    system.disconnect()
    return G1, G2


def read_models_prbs():
    with open(PATH_DATA + 'DCmotor_fo_model_pbrs.csv', newline='') as file:
        reader = csv.reader(file)
        # Iterate over each row in the CSV file
        num_line = 0
        for row in reader:
            if num_line != 0:
               alpha = float(row[0])
               tau = float(row[1])
            num_line += 1
    b = alpha / tau
    a = 1 / tau
    G1 = ct.tf(b, [1, a])

    with open(PATH_DATA + 'DCmotor_so_model_pbrs.csv', newline='') as file:
        reader = csv.reader(file)
        # Iterate over each row in the CSV file
        num_line = 0
        for row in reader:
            if num_line != 0:
                alpha1 = float(row[0])
                tau1 = float(row[1])
                tau2 = float(row[2])
            num_line += 1
    b0 = alpha1 / (tau1*tau2)
    a1 = (tau1 + tau2) / (tau1 * tau2)
    a0 = 1 / (tau1 * tau2)
    a = 1 / tau
    G1 = ct.tf(b, [1, a])
    G2 = ct.tf(b0, [1, a1, a0])
    return G1, G2

if __name__ == "__main__":
    motor1 = MotorSystemIoT()
    get_models_prbs(motor1)

