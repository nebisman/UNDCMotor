# Required libraries
from time import sleep

import numpy as np
import paho.mqtt.client as mqtt
import control as ct
import struct
from queue import Queue
import math
import json
import numpy as npy
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("TkAgg", force=True)

# parameters of communication

#BROKER = "broker.hivemq.com"
BROKER = "192.168.0.10"
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

U_ACT = [-5.0000, -4.8782, -4.7564, -4.6346, -4.5128, -4.3910, -4.2692, -4.1474, -4.0256, -3.9038,
         -3.7821, -3.6603, -3.5385, -3.4167, -3.2949, -3.1731, -3.0513, -2.9295, -2.8077, -2.6859,
         -2.5641, -2.4423, -2.3205, -2.1987, -2.0769, -1.9551, -1.8333, -1.7115, -1.5897, -1.4679,
         -1.3462, -1.2244, -1.1026, -0.9808, -0.8590, -0.7372, -0.6154, -0.4936, -0.3718, -0.2500,
         0.2500, 0.3718, 0.4936, 0.6154, 0.7372, 0.8590, 0.9808, 1.1026, 1.2244, 1.3462,
         1.4679, 1.5897, 1.7115, 1.8333, 1.9551, 2.0769, 2.1987, 2.3205, 2.4423, 2.5641,
         2.6859, 2.8077, 2.9295, 3.0513, 3.1731, 3.2949, 3.4167, 3.5385, 3.6603, 3.7821,
         3.9038, 4.0256, 4.1474, 4.2692, 4.3910, 4.5128, 4.6346, 4.7564, 4.8782, 5.0000
         ]

Y_ACT = [-2752.5001, -2744.7001, -2709.6002, -2691.6002, -2685.3002, -2639.5502, -2600.8501, -2564.2502, -2564.7002, -2551.9502,
         -2522.4002, -2503.2002, -2467.2001, -2423.5502, -2360.8502, -2376.7502, -2332.3502, -2284.0502, -2228.8502, -2194.6502,
         -2145.4501, -2103.4500, -2039.1002, -1975.3501, -1913.1002, -1830.3001, -1752.3002, -1657.0500, -1551.3001, -1439.7001,
         -1317.0001, -1166.1001, -1007.8501, -840.0001, -644.5501, -415.0500, -188.1000, -31.0500, -14.1000, -5.4000,
         0.0000, 15.6000, 39.7500, 284.7000, 540.6000, 770.7001, 960.9001, 1151.1001, 1318.3501, 1449.9001,
         1594.6501, 1730.8502, 1827.3001, 1908.9002, 1953.1500, 2012.7001, 2046.6002, 2122.6502, 2160.9001, 2226.4502,
         2236.8001, 2262.3001, 2319.9001, 2352.0002, 2376.3002, 2382.6001, 2415.1501, 2445.9001, 2425.9501, 2418.4501,
         2447.2501, 2457.9001, 2485.8001, 2572.6502, 2595.9001, 2611.6501, 2584.9502, 2605.3501, 2635.2002, 2652.3002
         ]

PBRS_LENGTH = 1023


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
    def actuator_gain(self, volts = 2.5):
        velocity = npy.interp(volts, U_ACT, Y_ACT)
        return velocity




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


def step_closed(system, low_val=0, high_val=90, low_time=1, high_time=1, filepath =r"./experiment_files/DCmotor_step_closed_exp.csv"):

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
def step_open(system, low_val=1.5, high_val=3.5, low_time=1, high_time=1, filepath = r"./experiment_files/DCmotor_step_open_exp.csv", visualize = True):
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

    if visualize == True:
        # Setting the graphics configuration for visualizing the experiment
        fig, (yax, uax) = plt.subplots(nrows=2, ncols=1, figsize=(12, 8))
        #fig, ax = plt.subplots()
        uax.grid(True);
        uax.grid(color='gray', linestyle='--', linewidth=0.25)
        yax.grid(True);
        yax.grid(color='gray', linestyle='--', linewidth=0.25)
        line_u, = uax.plot(t, u, drawstyle='steps-post', color="#338000")
        line_y, = yax.plot(t, y, drawstyle='steps-post',  color="#d40055")
        lim1 = low_val - 0.5
        lim2 = high_val + 0.5
        if lim2 >= lim1:
            uax.set_ylim(lim1 , lim2)
        else:
            uax.set_ylim(lim2, lim1)

        #ylimits = system.actuator_gain([low_val, high_val])
        ylimits = [0, 700]
        yax.set_ylim(ylimits[0] - 250, ylimits[1] + 250)
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
        tframe = sampling_time * (npy.arange(len(yframe)) + (curr_frame - 1) * buffer)

        for ind in range(len(yframe)):
            y.append(yframe[ind])
            u.append(uframe[ind])
            t.append(tframe[ind])
            exp.append([tframe[ind], uframe[ind], yframe[ind]])
            if visualize == True:
                line_u.set_data(t, u)
                line_y.set_data(t, y)
                plt.draw()
                plt.pause(0.005)

    npy.savetxt(filepath, exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,u,y')
    system.disconnect()
    plt.show()
    return t, u, y



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
            plt.draw()
            plt.pause(sampling_time)

    # Now, we save the results of the experiment in the provided filepath
    npy.savetxt(filepath, exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,r,y,u')
    # Now all is done, close the connection and close the figure.
    system.disconnect()
    return t, y, r, u



if __name__ == "__main__":
    motor1 = MotorSystemIoT()
    t = [0, 1, 2, 3, 4 , 5, 6, 40]
    y = [0, 1 , 1 , 2, 1, 1,0,1 ]
    y = [yi *720 for yi in y]
    set_pid(motor1, kp=0.02442626048, ki=0.0265210734635524428115, kd=0.00072572754865, N=11.9, beta=0.85)

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

    step_open(motor1, low_val= 0, high_val =5, low_time=0.5, high_time=0.5)

    #pbrs_open(motor1)
    #stairs_closed(motor1, signal, 4)


