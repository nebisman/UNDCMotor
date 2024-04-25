# Required libraries
from time import sleep
import numpy as np
from scipy.interpolate import PchipInterpolator, UnivariateSpline
import paho.mqtt.client as mqtt
import control as ct
import struct
from queue import Queue
import math
import json
from pathlib import Path
import csv
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("TkAgg", force=True)


# parameters of communication

#BROKER = "broker.hivemq.com"
BROKER = "192.168.0.3"
#BROKER = "18.204.70.207" # amazon mosquitto broker
# const char BROKER[] = "192.168.0.10";
PORT = 1883
USER = "hpdesktop"
PASSWORD = "hpdesktop"




#topics for subscribing

PLANT_NUMBER = "5678"
codes ={"SYS_USER_SIGNALS_CLOSED"  : "/motor/motor_" + PLANT_NUMBER + "/user/sig_closed",
        "SYS_USER_SIGNALS_OPEN"  : "/motor/motor_" + PLANT_NUMBER + "/user/sig_open",
        "USER_SYS_SET_REF"  : "/motor/user/motor_" + PLANT_NUMBER + "/set_ref",
        "USER_SYS_SET_PID"  : "/motor/user/motor_" + PLANT_NUMBER  + "/set_pid",
        "USER_SYS_STEP_CLOSED": "/motor/user/motor_" + PLANT_NUMBER +"/step_closed",
        "USER_SYS_STAIRS_CLOSED": "/motor/user/motor_" + PLANT_NUMBER + "/stairs_closed",
        "USER_SYS_PRBS_OPEN": "/motor/user/motor_" + PLANT_NUMBER + "/prbs_open",
        "USER_SYS_STEP_OPEN": "/motor/user/motor_" + PLANT_NUMBER + "/step_open",
        "USER_SYS_SET_GENCON": "/motor/user/motor_" + PLANT_NUMBER + "/set_gencon",
        "USER_SYS_PROFILE_CLOSED": "/motor/user/motor_" + PLANT_NUMBER + "/prof_closed",
        "MOTOR_SAMPLING_TIME" : 0.02,
        "BUFFER_SIZE" : 25,
        }


PBRS_LENGTH = 1023
PATH = r"./experiment_files/"

""" This is the class defining the IoT motor system"""
class MotorSystemIoT:

    def __init__(self, broker_address = BROKER, port= PORT, client_id="", clean_session=True):
        self.client = mqtt.Client()
        self.broker_address = broker_address
        self.port = port
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.on_subscribe = self.on_subscribe
        self.client.on_publish = self.on_publish
        self.codes = codes


    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Connected successfully to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print("Unexpected disconnection.")

    def on_message(self, client, userdata, message):
        print(f"Received  '{message.payload.decode()}'")

    def on_subscribe(self, client, userdata, mid, granted_qos):
        print("Subscribed: ", mid, " ", granted_qos)

    def on_publish(self, client, userdata, mid):
        print("Message Published: ", mid)

    def connect(self):
        self.client.username_pw_set(USER, PASSWORD)
        self.client.connect(self.broker_address, self.port)
        self.client.loop_start()

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()

    def subscribe(self, topic, qos=2):
        self.client.subscribe(topic, qos)

    def publish(self, topic, message, qos=1):
        self.client.publish(topic, message, qos)

    def transfer_function(self, output='position', min_order=True):
        if output == 'position':
            if min_order:
                num = [4369.0278678492]
                den = [1, 9.97077548888435, 0]
            else:
                num = [798289.164111307]
                den = [1, 187.699939287416, 1803.62183871806, 0]

        elif output == 'velocity':
            if min_order:
                num = [4369.0278678492]
                den = [1, 9.97077548888435]
            else:
                num = [798289.164111307]
                den = [1, 187.699939287416, 1803.62183871806]

        G = ct.tf(num, den)
        return G

    def speed_from_volts(self, volts = None):
        if volts == None:
            raise ValueError("voltage input is required")

        u, y = read_csv_file()
        interp = PchipInterpolator(u, y)
        return interp(volts)

    def volts_from_speed(self, speed = None):
        if speed == None:
            raise ValueError("speed input is required")

        u, y = read_csv_file()
        interp = PchipInterpolator(u, y)
        if speed == 0:
            uc = 0
        elif (speed >= y[0]) and (speed <= y[-1]):
            roots = interp.solve(speed, extrapolate=False)
            uc = np.mean(roots)
        else:
            raise ValueError(f"The speed input must be in the interval {y[0]} to {y[-1]}")
        return uc



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


def read_csv_file(filepath=PATH + 'static_gain_response.csv'):
    with open(filepath , newline='') as file:
        reader = csv.reader(file)
        # Iterate over each row in the CSV file
        num_line = 0
        u = []
        y = []
        for row in reader:
            if num_line != 0:
               u.append(float(row[0]))
               y.append(float(row[1]))
            num_line += 1
        return u, y


def set_reference(system, ref_value=50):
    ref_hex = float2hex(ref_value)
    topic_pub = system.codes["USER_SYS_SET_REF"]
    message = json.dumps({"reference": ref_hex})
    system.connect()
    system.publish(topic_pub, message)
    system.disconnect()
    rcode = True
    return rcode


def set_pid(system, kp=1, ki=0.4, kd=0, N=5, beta=1):
    topic_pub = system.codes["USER_SYS_SET_PID"]
    kp_hex = float2hex(kp)
    ki_hex = float2hex(ki)
    kd_hex = float2hex(kd)
    N_hex = float2hex(N)
    beta_hex = float2hex(beta)
    message = json.dumps({"kp": kp_hex,
                          "ki": ki_hex,
                          "kd": kd_hex,
                          "N": N_hex,
                          "beta": beta_hex})
    system.connect()
    system.publish(topic_pub, message)
    system.disconnect()
    rcode = True
    print("succesfull change of PID parameters")
    return rcode


def step_closed(system, low_val=0, high_val=90, low_time=1, high_time=1, filepath = PATH + "DCmotor_step_closed_exp.csv"):

    def step_message(system, userdata, message):
        # This inner function is the callback of the received messages
        q.put(message)

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
    frames = math.ceil(total_points/buffer)
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

    fig, ax = plt.subplots()
    ax.grid(True);
    ax.grid(color='gray', linestyle='--', linewidth=0.25)
    line_r, = ax.plot(t, r, drawstyle='steps-post', color="#338000") # drawstyle='steps'
    line_y, = ax.plot(t, y, drawstyle='steps-post', color="#d40055")
    ax.set_xlim(0, sampling_time * (total_points - 1))
    spany = high_val - low_val
    ax.set_ylim(low_val-0.95*spany, high_val + 0.95*spany)
    line_r.set_data(t, r)
    line_y.set_data(t, y)
    plt.draw()



    # this is the queue of messages filled by the step_message callback
    q = Queue()

    # at start we define a current frame of -1 indicating that no frame
    # has already been received
    curr_frame = -1
    while curr_frame < frames:
        try:
            message = q.get(True, 20* buffer * sampling_time)
        except:
            raise TimeoutError("The connection has been lost. Please try again")
        decoded_message = str(message.payload.decode("utf-8"))
        msg_dict = json.loads(decoded_message)
        frame_hex = str(msg_dict["frame"])
        curr_frame = hex2long(frame_hex)
        rframe_hex =  str(msg_dict["r"])
        uframe_hex = str(msg_dict["u"])
        yframe_hex = str(msg_dict["y"])
        rframe = hexframe_to_array(rframe_hex)
        uframe = hexframe_to_array(uframe_hex)
        yframe = hexframe_to_array(yframe_hex)
        tframe = sampling_time * (npy.arange(len(rframe)) + (curr_frame - 1) * buffer)

        for ind in range(len(rframe)):
            r.append(rframe[ind])
            y.append(yframe[ind])
            u.append(uframe[ind])
            t.append(tframe[ind])
            exp.append([tframe[ind], rframe[ind], yframe[ind], uframe[ind]])
            line_r.set_data(t, r)
            line_y.set_data(t, y)
            plt.draw()
            plt.pause(sampling_time)

    npy.savetxt(filepath, exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,r,y,u')
    system.disconnect()
    plt.show()
    return t, y, r, u


def stairs_closed(system, stairs=[40, 50, 60], duration= 2, filepath = r"./experiment_files/DCmotor_stairs_closed_exp.csv"):
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
    duration = math.ceil(duration / sampling_time)
    duration_hex = long2hex(duration)

    min_val = npy.min(stairs)
    max_val = npy.max(stairs)
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
    frames = math.ceil(total_points / buffer)


    # vectors for storing the results of the experiment
    y = []
    r = []
    u = []
    t = []
    exp = []

    # Setting the graphics configuration for visualizing the experiment

    fig, ax = plt.subplots(figsize=(12, 8))
    ax.grid(True);
    ax.grid(color='gray', linestyle='--', linewidth=0.25)
    line_r, = ax.plot(t, r, drawstyle='steps-post', color="#338000")
    line_y, = ax.plot(t, y, drawstyle='steps-post', color="#d40055")
    ax.set_xlim(0, sampling_time * (total_points - 1))
    min_val = npy.min(stairs)
    max_val = npy.max(stairs)
    spany = max_val - min_val
    ax.set_ylim( min_val-0.1*abs(spany), max_val + 0.1* spany)
    line_r.set_data(t, r)
    line_y.set_data(t, y)
    plt.draw()

    # This is the queue of messages filled by the stair_message callback
    q = Queue()

    # At beginning we define a current frame of -1 indicating that no frame
    # has already been received
    curr_frame = -1

    # loop for receiving dataframes from the ESP32

    while curr_frame < frames:
        try:
            # we wait for 10 seconds for a new dataframe
            message = q.get(True, 20 * buffer * sampling_time)
        except:
            # else we raise a communication error
            raise TimeoutError("The connection has been lost. Please try again")

        # decoding the message
        decoded_message = str(message.payload.decode("utf-8"))
        msg_dict = json.loads(decoded_message)
        frame_hex = str(msg_dict["frame"])
        curr_frame = hex2long(frame_hex)
        rframe_hex = str(msg_dict["r"])
        uframe_hex = str(msg_dict["u"])
        yframe_hex = str(msg_dict["y"])
        rframe = hexframe_to_array(rframe_hex)
        uframe = hexframe_to_array(uframe_hex)
        yframe = hexframe_to_array(yframe_hex)
        tframe = sampling_time * (npy.arange(len(rframe)) + (curr_frame - 1) * buffer)

        # we plot every single point received in each dataframe
        # and save it in the matrix exp for storing in a csv file
        for ind in range(len(rframe)):
            #storing t, r, y, and u vectors
            r.append(rframe[ind])
            y.append(yframe[ind])
            u.append(uframe[ind])
            t.append(tframe[ind])

            # storing the experiment
            exp.append([tframe[ind], rframe[ind], yframe[ind], uframe[ind]])
            line_r.set_data(t, r)
            line_y.set_data(t, y)

            # drawing a new point from the current dataframe
            plt.draw()
            plt.pause(sampling_time)

    # Now, we save the results of the experiment in the provided filepath
    npy.savetxt(filepath, exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,r,y,u')
    # Now all is done, close the connection and close the figure.
    system.disconnect()
    return t, y, r, u



def pbrs_open(system, low_val = 2, high_val = 4, divider = 2,  filepath = r"./experiment_files/DCmotor_prbs_open_exp.csv"):
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
    frames = math.ceil(total_points/buffer)
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
    fig, (yax, uax) = plt.subplots(nrows=2, ncols=1, figsize=(12, 8))
    #fig, ax = plt.subplots()
    uax.grid(True);
    uax.grid(color='gray', linestyle='--', linewidth=0.25)
    yax.grid(True);
    yax.grid(color='gray', linestyle='--', linewidth=0.25)
    line_u, = uax.plot(t, u, drawstyle='steps-post', color="#338000")
    line_y, = yax.plot(t, y,  color="#d40055")
    uax.set_ylim(low_val - 0.5  , high_val + 0.5 )
    limits = [600,800]#system.actuator_gain([low_val/3, high_val/3])
    yax.set_ylim(limits[0], limits[1])

    # setting the parameters of the step response for sending to ESP32
    fig.suptitle(f'Experiment of System identification with {total_points:d} samples and duration of {total_points*sampling_time: 0.2f} seconds')
    uax.set_title('Input voltage to DC motor')
    yax.set_title('Velocity in degrees per second')
    uax.set_xlabel('Time (s)')
    uax.set_ylabel('Volts')
    yax.set_ylabel('Degrees/s')

    # this is the queue of messages filled by the step_message callback
    q = Queue()

    # at start we define a current frame of -1 indicating that no frame
    # has already been received
    curr_frame = -1

    # we keep the loop until all frames have been receuves
    while curr_frame < frames:
        try:
            # We try to extract the last payload from the queue, and if it is empty
            # we wait for at least 10 seconds
            message = q.get(True, 20 * buffer * sampling_time)
        except:
            # after ten seconds we raise a timeout
            raise TimeoutError("The connection has been lost. Please try again")

        # this es the current received  message
        decoded_message = str(message.payload.decode("utf-8"))
        msg_dict = json.loads(decoded_message)
        # reading the current frame's number
        frame_hex = str(msg_dict["frame"])
        curr_frame = hex2long(frame_hex)
        # reading the current frame values for the input u and the output y
        uframe_hex = str(msg_dict["u"])
        yframe_hex = str(msg_dict["y"])
        uframe = hexframe_to_array(uframe_hex)
        yframe = hexframe_to_array(yframe_hex)

        # Defining the current frame of time
        tframe = sampling_time * (npy.arange(len(uframe)) + (curr_frame - 1) * buffer)

        # updating t,u, and y vectors
        y.extend(yframe)
        u.extend(uframe)
        t.extend(tframe)

        #we plot one frame at frame 1
        if curr_frame == 1:
            uax.set_xlim(t[0], t[-1])
            yax.set_xlim(t[0], t[-1])

        # after frame 1 we plot the last two frames
        elif curr_frame >1:
            uax.set_xlim(t[-2*buffer], t[-1])
            yax.set_xlim(t[-2*buffer], t[-1])


        # plotting the current data
        line_u.set_data(t, u)
        line_y.set_data(t, y)
        plt.draw()
        plt.pause(0.1)

    # preparing the matrix for storing the results of the identification experiment
    for ind in range(len(y)):
        exp.append([t[ind],  u[ind], y[ind]])

    # Saving the results of the experiment, stored in list 'exp', in the given filepath
    npy.savetxt(filepath, exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,u,y')
    system.disconnect()
    return t, u, y
#
#
def step_open(system, low_val=1.5, high_val=3.5, low_time=1, high_time=1, filepath = r"./experiment_files/DCmotor_step_open_exp.csv"):
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
    frames = math.ceil(total_points / buffer)
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
    fig, (yax, uax) = plt.subplots(nrows=2, ncols=1, figsize=(12, 8))

    uax.grid(True);
    uax.grid(color='gray', linestyle='--', linewidth=0.25)
    yax.grid(True);
    yax.grid(color='gray', linestyle='--', linewidth=0.25)
    line_u, = uax.plot(t, u, drawstyle='steps-post', color="#338000")
    line_y, = yax.plot(t, y, drawstyle='steps-post',  color="#d40055")


    if  high_val  >=  low_val:
        ulimits = [low_val, high_val]
        ylimits = system.speed_from_volts([low_val, high_val])
    else:
        ulimits = [high_val, low_val]
        ylimits = system.speed_from_volts([high_val, low_val])

    percent = 0.2
    uax.set_ylim((1 - percent)*ulimits[0], (1 + percent)*ulimits[1])
    yax.set_ylim((1 - percent)*ylimits[0], (1 + percent)*ylimits[1])
    uax.set_xlim(0, sampling_time * (total_points - 1))
    yax.set_xlim(0, sampling_time * (total_points - 1))

    fig.suptitle(f'Experiment of Step response with a duration of {total_points*sampling_time: 0.2f} seconds')
    uax.set_title('Input voltage to DC motor')
    yax.set_title('Velocity in degrees per second')
    uax.set_xlabel('Time (s)')
    uax.set_ylabel('Volts')
    yax.set_ylabel('Degrees/s')
    line_u.set_data(t, u)
    line_y.set_data(t, y)
    plt.draw()

    # this is the queue of messages filled by the step_message callback
    q = Queue()

    # at start we define a current frame of -1 indicating that no frame
    # has already been received
    curr_frame = -1
    while curr_frame < frames:
        try:
            message = q.get(True, 20* buffer * sampling_time)
        except:
            raise TimeoutError("The connection has been lost. Please try again")
        decoded_message = str(message.payload.decode("utf-8"))
        msg_dict = json.loads(decoded_message)
        frame_hex = str(msg_dict["frame"])
        curr_frame = hex2long(frame_hex)
        uframe_hex = str(msg_dict["u"])
        yframe_hex = str(msg_dict["y"])
        uframe = hexframe_to_array(uframe_hex)
        yframe = hexframe_to_array(yframe_hex)
        tframe = sampling_time * (np.arange(len(yframe)) + (curr_frame - 1) * buffer)

        for ind in range(len(yframe)):
            y.append(yframe[ind])
            u.append(uframe[ind])
            t.append(tframe[ind])
            exp.append([tframe[ind], uframe[ind], yframe[ind]])
            line_u.set_data(t, u)
            line_y.set_data(t, y)
            plt.draw()
            plt.pause(0.005)
    plt.close()
    np.savetxt(filepath, exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,u,y')
    system.disconnect()
    return t, u, y



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
    frames = math.ceil(total_points / buffer)
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

    # at start we define a current frame of -1 indicating that no frame
    # has already been received
    curr_frame = -1
    while curr_frame < frames:
        try:
            message = q.get(True, 20* buffer * sampling_time)
        except:
            raise TimeoutError("The connection has been lost. Please try again")
        decoded_message = str(message.payload.decode("utf-8"))
        msg_dict = json.loads(decoded_message)
        frame_hex = str(msg_dict["frame"])
        curr_frame = hex2long(frame_hex)
        uframe_hex = str(msg_dict["u"])
        yframe_hex = str(msg_dict["y"])
        uframe = hexframe_to_array(uframe_hex)
        yframe = hexframe_to_array(yframe_hex)
        y.extend(yframe)
        u.extend(uframe)

    return u, y




def set_controller(system, controller):
    topic_pub = system.codes["USER_SYS_SET_GENCON"]
    sampling_time = system.codes["MOTOR_SAMPLING_TIME"]
    Cvecont = ct.tf2ss(controller)
    Cve = ct.c2d(Cvecont, sampling_time, method='tustin')
    A = Cve.A
    B = Cve.B
    Cc = Cve.C
    D = Cve.D
    order = len(A)
    P = [(0.6 + 0.001*i) for i in range(order)]
    L = ct.place(npy.transpose(A), npy.transpose(Cc), P)
    L = npy.transpose(L)
    Ac = A - L * Cc
    Bc = B - L * D
    if (npy.size(Bc, axis=1)) == 1:
        B1 = []
        for row in Bc:
            for e in row:
                B1.append([e, -e])
        Bc = npy.array(B1)
        Dc = npy.array([[D[0][0], -D[0][0]]])
    A_hex = matrix2hex(Ac)
    B_hex = matrix2hex(Bc)
    C_hex = matrix2hex(Cc)
    D_hex = matrix2hex(Dc)
    L_hex = matrix2hex(L)
    order_hex = long2hex(order)
    message = json.dumps({"order": order_hex,
                          "A": A_hex,
                          "B": B_hex,
                          "C": C_hex,
                          "D": D_hex,
                          "L": L_hex
                          })

    system.connect()
    system.publish(topic_pub, message)
    system.disconnect()
    rcode = True
    return rcode



def profile_closed(system, timevalues = [0, 1, 2 ,3], refvalues = [0, 720, 720, 0], filepath = r"./experiment_files/DCmotor_profile_closed_exp.csv"):
    def profile_message(system, userdata, message):
        # This is the callback for receiving messages from the plant
        q.put(message)

    # reading the configuration parameters from the code's field in the plant

    topic_pub = system.codes["USER_SYS_PROFILE_CLOSED"]
    topic_sub = system.codes["SYS_USER_SIGNALS_CLOSED"]
    sampling_time = system.codes["MOTOR_SAMPLING_TIME"]
    buffer = system.codes["BUFFER_SIZE"]

    # setting the parameters of the step response for sending to ESP32

    int_timevalues = [round(p/0.02) for p in timevalues]
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
    min_val = npy.min(refvalues)
    max_val = npy.max(refvalues)
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
    frames = math.ceil(total_points / buffer)

    # vectors for storing the results of the experiment
    y = []
    r = []
    u = []
    t = []
    exp = []

    # Setting the graphics configuration for visualizing the experiment

    fig, ax = plt.subplots(figsize=(12, 8))
    ax.grid(True);
    ax.grid(color='gray', linestyle='--', linewidth=0.25)
    line_r, = ax.plot(t, r, drawstyle='steps-post', color="#338000")
    line_y, = ax.plot(t, y, drawstyle='steps-post', color="#d40055")
    ax.set_xlim(0, sampling_time * (total_points - 1))
    spany = max_val - min_val
    ax.set_ylim( min_val-0.1*abs(spany), max_val + 0.1* spany)
    line_r.set_data(t, r)
    line_y.set_data(t, y)
    plt.draw()

    # This is the queue of messages filled by the stair_message callback
    q = Queue()

    # At beginning we define a current frame of -1 indicating that no frame
    # has already been received
    curr_frame = -1

    # loop for receiving dataframes from the ESP32
    while curr_frame < frames:
        try:
            # we wait for 10 seconds for a new dataframe
            message = q.get(True, 20 * buffer * sampling_time)
        except:
            # else we raise a communication error
            raise TimeoutError("The connection has been lost. Please try again")

        # decoding the message
        decoded_message = str(message.payload.decode("utf-8"))
        msg_dict = json.loads(decoded_message)
        frame_hex = str(msg_dict["frame"])
        curr_frame = hex2long(frame_hex)
        rframe_hex = str(msg_dict["r"])
        uframe_hex = str(msg_dict["u"])
        yframe_hex = str(msg_dict["y"])
        rframe = hexframe_to_array(rframe_hex)
        uframe = hexframe_to_array(uframe_hex)
        yframe = hexframe_to_array(yframe_hex)
        tframe = sampling_time * (npy.arange(len(rframe)) + (curr_frame - 1) * buffer)

        # we plot every single point received in each dataframe
        # and save it in the matrix exp, which allows to write a csv file
        for ind in range(len(rframe)):
            #storing t, r, y, and u vectors
            r.append(rframe[ind])
            y.append(yframe[ind])
            u.append(uframe[ind])
            t.append(tframe[ind])

            # storing the experiment
            exp.append([tframe[ind], rframe[ind], yframe[ind], uframe[ind]])
            line_r.set_data(t, r)
            line_y.set_data(t, y)

            # drawing a new point from the current dataframe
            plt.show()
            plt.pause(sampling_time)

    # Now, we save the results of the experiment in the provided filepath
    npy.savetxt(filepath, exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,r,y,u')
    # Now all is done, close the connection and close the figure.
    system.disconnect()
    return t, y, r, u



def get_static_model(system, lowval = 1.5):
    def moving_average(signal):
        n = 25
        """Calculate the moving average filter of a signal with order n."""
        weights = np.ones(n) / n
        return np.convolve(signal, weights, mode='valid')[-1]

    # This is the configuration for the figure displayed while acquiring data
    yee = []
    uee = []

    fig, ax = plt.subplots(figsize=(12, 8))
    fig.set_facecolor('#b7c8be')
    ax.set_title('Static gain response experiment for UNDCMotor')
    ax.set_xlabel('Input (Volts)')
    ax.set_ylabel('Steady state speed (Degrees/s)')

    ax.grid(True);
    ax.set_facecolor('#f4eed7')
    ax.set_xticks([-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5])
    ax.grid(color='#1a1a1a40', linestyle='--', linewidth=0.25)
    line_exp, = ax.plot(uee, yee, color="#00aa00", linewidth=1.5)
    ax.set_xlim(-5, 5)
    ax.set_ylim(-780, 780)
    plt.draw()

    # These are the parameters for obtaining the actuator response
    timestep = 3
    points = 30
    dz_point = 0.32

    # # This is the set of step responses for obtaining steady state in speed
    delta_dz = 0.02
    u_pos = np.logspace(np.log10(dz_point - delta_dz), np.log10(5), points , endpoint=True, base=10)
    u_neg = -u_pos[::-1]
    u_tot = np.concatenate((u_neg, [0], u_pos))
    exp = []

    for ui in u_tot:
        lowval_turn = np.sign(ui) * lowval
        if ui == 0:
            uf = 0
            yf = 0
        else:
            print(lowval)
            u, y = step_open_staticgain(system, lowval_turn, ui, 0.5, timestep)
            yf = moving_average(y)
            uf = u[-1]
        exp.append([uf, yf])
        yee.append(yf)
        uee.append(ui)
        line_exp.set_data(uee, yee)
        plt.draw()
        plt.pause(0.1)

    Path(PATH).mkdir(exist_ok=True)
    np.savetxt(PATH + "static_gain_response.csv", exp, delimiter=",", fmt="%0.8f", comments="", header='u,y')
    system.disconnect()
    return

def get_fomodel_step(system, speed=500, timestep=2.5):
    """First we estimate the gain of the model using
     the mean value theorem for derivatives"""

    # We calculate voltages around the operation point
    # given a  deviation percent
    speed = np.clip(speed, -770, 770)
    percent = 0.15
    u = system.volts_from_speed(speed)
    ua = (1 - percent) * u
    ub = (1 + percent) * u
    if abs(speed) < 220:
        ua = 0.45 * np.sign(speed)
        ub = 0.35 *  np.sign(speed)



    # Obtaining the step response of the motor, where the low level of the step
    # and the high level last the time given by timestep

    t, u, y = step_open(system, low_val=ua, high_val=ub, low_time=timestep, high_time=timestep)

    # we interpolate the experimental response
    interp = PchipInterpolator(t, y)

    # we estimate the steady state speeds achieved during the low level of the step.
    ta = [t0 for t0 in t if t0 > timestep - 1 and t0 < timestep]
    ya = np.mean(interp(ta))

    # we estimate the steady state speeds achieved during the high level of the step.
    tb = [t0 for t0 in t if t0 > timestep + 1 and t0 < 2 * timestep]
    yb = np.mean(interp(tb))

    # we use the mean value theorem to approximate the gain of the motor
    delta_y = yb - ya
    delta_u = ub - ua
    alpha = delta_y / delta_u

    # the constant tau in the first order systems occurs when the speed response
    # changes by 63%

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

    # we take the mean for the case that the noise produces multiple values.
    # we also need to substract the time in which the step changes

    t1  = np.mean(roots_t1) - timestep
    t2  =  np.mean(roots_t2) - timestep
    tau3 = np.mean(roots_t3) - timestep
    #t4 =  np.mean(roots_t4) - timestep

    tau1 = t1 / 0.2231
    tau2 = t2 / 0.5108

    #tau4 = t4 /1.6094

    tau = (tau1 + tau2 + tau3)/3



    # we build the model
    G = ct.tf(alpha, [tau, 1])

    # we calculate the step response from the model
    um = np.array(u) - u[0]  # it is required to compute the LTI model with a signal starting in 0.
    tm, ym = ct.forced_response(G, t, um)
    ym = ym + ya

    # now we compare the model with the experimental data
    fig, (ay, au) = plt.subplots(nrows=2, ncols=1, width_ratios = [1], height_ratios= [4,1], figsize=(12, 8))
    fig.set_facecolor('#b7c4c8f0')
    ay.set_title('Estimated first order model for UNDCMotor')
    ay.set_ylabel('Speed (Degrees/s)')
    ay.grid(True);
    ay.grid(color='#1a1a1a40', linestyle='--', linewidth=0.25)
    ay.set_facecolor('#f4eed7')
    ay.set_xlim(0, 2*timestep)
    box = dict(boxstyle='round,pad=0.5', facecolor='white', edgecolor='white', alpha=0.5)
    ay.text(4, (ya+yb)/2, r'$\Delta_{y,e}=%0.2f$'%delta_y, fontsize=16, color='#ff0066',
             ha='center', va='bottom', bbox=box)
    ay.text(timestep + tau + 0.2, ya + 0.63212*delta_y,  r'$\tau = %0.2f$'%tau, fontsize=16, color='#ff0066')

    au.set_xlim(0, 2 *timestep)
    au.grid(True);
    au.set_facecolor('#d7f4ee')
    au.grid(color='#1a1a1a40', linestyle='--', linewidth=0.25)
    au.text(4, (ua+ub)/2, r'$\Delta_u=%0.2f$'%delta_u, fontsize=16, color="#00aa00",
             ha='center', va='bottom', bbox=box)
    au.set_xlabel('Time (seconds)')
    ay.set_ylabel('Voltage (V)')
    line_exp, = ay.plot(t, y, color="#0088aa", linewidth=1.5, linestyle=(0, (1, 1)))
    line_mod, = ay.plot(tm, ym, color="#ff0066", linewidth=1.5, )
    ay.plot(timestep + tau, ya + 0.63212*delta_y , color="#ff0066", linewidth=1.5, marker=".", markersize=13)
    line_u, = au.plot(t, u, color="#00aa00")
    modelstr = r"Model $G(s)= \frac{\alpha_m}{\tau_m\,s + 1} = \frac{%0.3f }{%0.3f\,s+1}$" %(alpha, tau)

    ay.legend([line_exp, line_mod], ['Data', modelstr], fontsize=16)
    au.legend([line_u], ['Input'])
    PATH1 = r'/home/leonardo/sharefolder/ProyectoSabatico/Reporte/figures/'
    #plt.savefig(PATH1 + 'first_model_response.svg', format='svg', bbox_inches='tight')
    plt.show()

    Path(PATH).mkdir(exist_ok=True)
    exp = [[alpha, tau]]

    np.savetxt(PATH + "first_order_model.csv", exp, delimiter=",", fmt="%0.8f", comments="", header='alpha, tau')

    system.disconnect()
    return G



if __name__ == "__main__":
    motor1 = MotorSystemIoT()
    #val = motor1.volts_from_speed(200)
    print(get_fomodel_step(motor1,400))
    # print(motor1.volts_from_speed(1.15*400))
    # print(motor1.volts_from_speed(0.85*400))





    #u = 1
    #motor1.get_steady_state_curve(lowval=1.5)


    # t = [0, 1, 2, 3, 4 , 5, 6, 40]
    # y = [0, 1 , 1 , 2, 1, 1,0,1 ]
    # y = [yi *720 for yi in y]
    # set_pid(motor1, kp=0.02442626048, ki=0.0265210734635524428115, kd=0.00072572754865, N=11.9, beta=0.85)

    #profile_closed(motor1, t, y )

    #signal = [0, 45, 90, 135, 180, 135, 90, 45, 0]
    #stairs_closed(motor1 , signal, 2)
    # u = np.linspace(-5,5,100);
    # y=motor1.actuator_gain(u)
    # plt.plot(u,y)
    # plt.show()


    #stairs_closed(motor1, signal, 1.5)
    # #print(plant.transfer_function(output='velocity'))
    #set_pid(motor1, kp=0.026048, ki=0.028115, kd=0.00074865, N=11.9, beta=0.9)
    #sleep(1)

    #step_closed(motor1, low_val=0, high_val=90, low_time=1, high_time=2)

    #step_open(motor1, low_val= 0, high_val =5, low_time=1, high_time=1)

    #pbrs_open(motor1)
    #stairs_closed(motor1, signal, 4)


    # def get_model_from_step(system, speed = 500, timestep = 2.5):
    #     """First we estimate the gain of the model using
    #      the mean value theorem for derivatives"""
    #
    #     # We calculate voltages around the operation point
    #     # given a  deviation percent
    #     percent = 0.15
    #     u = system.volts_from_speed(speed)
    #     ua = (1 - percent)*u
    #     ub = (1 + percent)*u
    #
    #     # Obtaining the step response of the motor, where the low level of the step
    #     # and the high level last the time given by timestep
    #
    #     t, u, y = step_open(system, low_val=ua, high_val=ub, low_time=timestep, high_time=timestep)
    #
    #     # we interpolate the experimental response
    #     interp = PchipInterpolator(t, y)
    #
    #     # we estimate the steady state speeds achieved during the low level of the step.
    #     ta = [t0 for t0 in t if t0> timestep - 1 and t0 < timestep]
    #     ya = np.mean(interp(ta))
    #
    #     # we estimate the steady state speeds achieved during the high level of the step.
    #     tb = [t0 for t0 in t if t0 > timestep + 1 and t0 < 2 * timestep]
    #     yb = np.mean(interp(tb))
    #
    #     # we use the mean value theorem to approximate the gain of the motor
    #     delta_y = yb - ya
    #     delta_u = ub - ua
    #     alpha = delta_y / delta_u
    #
    #     # the constant tau in the first order systems occurs when the speed response
    #     # changes by 63%
    #     y_tau = ya + 0.63212*delta_y
    #
    #     # with this value, we can approximate the value of tau
    #     # solving the inverse equation by means of the interpolator
    #     roots = interp.solve(y_tau, extrapolate=False )
    #
    #     # we take the mean for the case that the noise produces multiple values.
    #     # we also need to substract the time in which the step changes
    #     tau = np.mean(roots)  - timestep
    #
    #     # we build the model
    #     G = ct.tf(alpha, [tau, 1])
    #
    #     # we calculate the step response from the model
    #     um = np.array(u)-u[0] # it is required to compute the LTI model with a signal starting in 0.
    #     tm, ym  = ct.forced_response(G, t, um)
    #     ym = ym + ya
    #     # we compare the model with the experimental data
    #     fig, ax = plt.subplots(figsize=(12, 8))
    #     fig.set_facecolor('#b7c4c8f0')
    #     ax.set_title('Estimated first order model for UNDCMotor')
    #     ax.set_xlabel('Time (seconds)')
    #     ax.set_ylabel('Speed (Degrees/s)')
    #     ax.grid(True);
    #     ax.set_facecolor('#f4eed7')
    #     #ax.set_xticks([-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5])
    #     ax.grid(color='#1a1a1a40', linestyle='--', linewidth=0.25)
    #     line_exp, = ax.plot(t, y, color="#0088aa", linewidth=1.5 , linestyle=(0, (1, 1)))
    #     line_mod, = ax.plot(tm, ym, color="#ff0066", linewidth=1.5,)
    #     plt.show()
    #     plt.legend()
    #     Path(PATH).mkdir(exist_ok=True)
    #     exp = [[alpha, tau]]
    #     np.savetxt(PATH + "first_order_model.csv", exp, delimiter=",", fmt="%0.8f", comments="", header='alpha, tau')
    #     self.disconnect()
    #     return
