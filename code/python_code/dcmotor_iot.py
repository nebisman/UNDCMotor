# Required libraries
from time import sleep
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


BROKER = "192.168.0.8"
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
        "MOTOR_SAMPLING_TIME" : 0.02,
        "BUFFER_SIZE" : 25,
        }

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


def step_closed(system, low_val=0, high_val=90, low_time=1, high_time=1, filepath ="DCmotor_step_closed_exp.csv"):

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

    points_high = round(high_time / sampling_time) + 1
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
    line_r, = ax.plot(t, r, drawstyle='steps', color="#338000") # drawstyle='steps'
    line_y, = ax.plot(t, y, drawstyle='steps', color="#d40055")
    ax.set_xlim(0, sampling_time * (total_points - 1))
    spany = high_val - low_val
    ax.set_ylim(max(0,low_val-0.9*spany), high_val + 0.95*spany)
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
        # r.extend(rframe)
        # u.extend(uframe)
        # y.extend(yframe)
        # t.extend(tframe)
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


def stairs_closed(system, stairs=[40, 50, 60], duration=100, filepath = "stairs_closed_exp.csv"):
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
    message = json.dumps({"signal": signal_hex, "duration": duration_hex,
                          "points_stairs": points_stairs_hex})

    # command sent to ESP32 for obtaining the step response
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

    fig, ax = plt.subplots()
    ax.grid(True);
    ax.grid(color='gray', linestyle='--', linewidth=0.25)
    line_r, = ax.plot(t, r, drawstyle='steps', color="#338000")  # drawstyle='steps'
    line_y, = ax.plot(t, y, drawstyle='steps', color="#d40055")
    ax.set_xlim(0, sampling_time * (total_points - 1))
    min_val = npy.min(signal)
    max_val = npy.max(signal)
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

    # looṕ for receivind dataframes from the ESP32
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
    plt.show()
    return t, y, r, u


# #
#
# def pbrs_open(system, op_point=50, peak_amp=5, stab_time=150, uee_time=20, divider=35, filepath = "prbs_open_exp.csv"):
#     def pbrs_message(system, userdata, message):
#         q.put(message)
#
#     topic_pub = system.codes["USER_SYS_PRBS_OPEN"]
#     topic_sub = system.codes["SYS_USER_SIGNALS_OPEN"]
#     sampling_time = system.codes["THERMAL_SAMPLING_TIME"]
#     peak_amp_hex = float2hex(peak_amp)
#     op_point_hex = float2hex(op_point)
#     stab_points = math.ceil(stab_time / sampling_time)
#     uee_points = math.ceil(uee_time / sampling_time)
#     stab_points_hex = long2hex(stab_points)
#     uee_points_hex = long2hex(uee_points)
#     divider_hex = long2hex(divider)
#     points = divider * 63 + stab_points + uee_points
#     message = json.dumps({"peak_amp": peak_amp_hex,
#                           "op_point": op_point_hex,
#                           "stab_points": stab_points_hex,
#                           "uee_points": uee_points_hex,
#                           "divider": divider_hex
#                           })
#     system.client.on_message = pbrs_message
#     system.connect()
#     system.subscribe(topic_sub)
#     system.publish(topic_pub, message)
#     q = Queue()
#     np = None
#     y = []
#     u = []
#     t = []
#     yt = []
#     ut = []
#     tt = []
#     exp = []
#     fig, ax = plt.subplots()
#     line_y, = ax.plot(t, y, color="#ffcc00")
#     line_u, = ax.plot(t, u, color="#00d4aa")
#     line_yt, = ax.plot(t, yt, color="#d40055")
#     line_ut, = ax.plot(t, ut, color="#338000")
#     a1 = 0.971914417613643
#     a2 = -24.7354557071619
#     uf_est = a1 * op_point + a2
#     Tmax_est = (uf_est + peak_amp - a2) / a1
#     Tmin_est = (uf_est - peak_amp - a2) / a1
#     umin_est = a1 * Tmin_est + a2
#     ax.set_xlim(0, sampling_time * points)
#     ax.set_ylim(umin_est - 5, Tmax_est + 5)
#     plt.grid()
#     npc = 1
#     while npc <= points:
#         try:
#             message = q.get(True, 20 * sampling_time)
#         except:
#             raise TimeoutError("The connection has been lost. Please try again")
#
#         decoded_message = str(message.payload.decode("utf-8"))
#         msg_dict = json.loads(decoded_message)
#         np_hex = str(msg_dict["np"])
#         np = hex2long(np_hex)
#         print(points, npc, np, hex2float(msg_dict["u"]), hex2float(msg_dict["y"]))
#         if np == npc:
#             if npc <= stab_points + uee_points:
#                 t_curr = (np - 1) * sampling_time
#                 t.append(t_curr)
#                 y_curr = hex2float(msg_dict["y"])
#                 y.append(y_curr)
#                 u_curr = hex2float(msg_dict["u"])
#                 u.append(u_curr)
#                 line_y.set_data(t, y)
#                 line_u.set_data(t, u)
#                 if npc == stab_points + uee_points:
#                     tt.append(t_curr)
#                     yt.append(y_curr)
#                     ut.append(u_curr)
#
#                     #exp.append([0, u_curr, y_curr])
#             else:
#                 tt_curr = (np - 1) * sampling_time
#                 if npc == stab_points + uee_points + 1:
#                     t0 = t_curr
#                 tt.append(tt_curr)
#                 yt_curr = hex2float(msg_dict["y"])
#                 yt.append(yt_curr)
#                 ut_curr = hex2float(msg_dict["u"])
#                 ut.append(ut_curr)
#                 line_yt.set_data(tt, yt)
#                 line_ut.set_data(tt, ut)
#                 line_y.set_data(t, y)
#                 line_u.set_data(t, u)
#                 exp.append([tt_curr-t0, ut_curr, yt_curr])
#             plt.draw()
#             plt.pause(0.1)
#             npc += 1
#     npy.savetxt(filepath, exp, delimiter=",",
#                 fmt="%0.8f", comments="", header='t,u,y')
#     system.disconnect()
#     plt.show()
#     return t, u, y
#
#
# def step_open(system, op_point=50, amplitude=5, high_time=200, stab_time=150, uee_time=20, filepath = "step_open_exp.csv"):
#     def step_message(system, userdata, message):
#         q.put(message)
#
#     topic_pub = system.codes["USER_SYS_STEP_OPEN"]
#     topic_sub = system.codes["SYS_USER_SIGNALS_OPEN"]
#     sampling_time = system.codes["THERMAL_SAMPLING_TIME"]
#     amp_hex = float2hex(amplitude)
#     points_high = math.ceil(high_time / sampling_time)
#     points_high_hex = long2hex(points_high)
#     op_point_hex = float2hex(op_point)
#     stab_points = math.ceil(stab_time / sampling_time)
#     uee_points = math.ceil(uee_time / sampling_time)
#     stab_points_hex = long2hex(stab_points)
#     uee_points_hex = long2hex(uee_points)
#     message = json.dumps({"amplitude": amp_hex,
#                           "op_point": op_point_hex,
#                           "stab_points": stab_points_hex,
#                           "uee_points": uee_points_hex,
#                           "points_high": points_high_hex,
#                           })
#     system.client.on_message = step_message
#     system.connect()
#     system.subscribe(topic_sub)
#     system.publish(topic_pub, message)
#     q = Queue()
#     np = None
#     y = []
#     u = []
#     t = []
#     yt = []
#     ut = []
#     tt = []
#     exp =[]
#     fig, ax = plt.subplots()
#     line_y, = ax.plot(t, y, color="#ffcc00")
#     line_u, = ax.plot(t, u, color="#00d4aa")
#     line_yt, = ax.plot(t, yt, color="#d40055")
#     line_ut, = ax.plot(t, ut, color="#338000")
#
#     a1 = 0.971914417613643
#     a2 = -24.7354557071619
#     uf_est = a1 * op_point + a2
#     points = stab_points + uee_points + points_high
#     Tmax_est = (uf_est + amplitude - a2) / a1
#     ax.set_xlim(0, sampling_time * points)
#     ax.set_ylim(uf_est - 5, Tmax_est + 5)
#     plt.grid()
#     npc = 1
#     while npc <= points:
#         try:
#             message = q.get(True, 20 * sampling_time)
#         except:
#             raise TimeoutError("The connection has been lost. Please try again")
#
#         decoded_message = str(message.payload.decode("utf-8"))
#         msg_dict = json.loads(decoded_message)
#         np_hex = str(msg_dict["np"])
#         np = hex2long(np_hex)
#         print(points, npc, np, hex2float(msg_dict["u"]), hex2float(msg_dict["y"]))
#         if np == npc:
#             if npc <= stab_points + uee_points:
#                 t_curr = (np - 1) * sampling_time
#                 t.append(t_curr)
#                 y_curr = hex2float(msg_dict["y"])
#                 y.append(y_curr)
#                 u_curr = hex2float(msg_dict["u"])
#                 u.append(u_curr)
#                 line_y.set_data(t, y)
#                 line_u.set_data(t, u)
#
#             else:
#
#                 tt_curr = (np - 1) * sampling_time
#                 if npc <= stab_points + uee_points + 1:
#                     t0 = tt_curr
#                 tt.append(tt_curr)
#                 yt_curr = hex2float(msg_dict["y"])
#                 yt.append(yt_curr)
#                 ut_curr = hex2float(msg_dict["u"])
#                 ut.append(ut_curr)
#                 exp.append([tt_curr - t0, ut_curr, yt_curr])
#                 line_yt.set_data(tt, yt)
#                 line_ut.set_data(tt, ut)
#                 line_y.set_data(t, y)
#                 line_u.set_data(t, u)
#             plt.draw()
#             plt.pause(0.1)
#             npc += 1
#     npy.savetxt(filepath, exp, delimiter=",",
#                 fmt="%0.8f", comments="", header='t,u,y')
#     system.disconnect()>
#     plt.show()
#     return t, u, y
#
#
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




# hexlist = "c2b40000,c2b40000,42b40000,42b40000,42b40000,42b40000,42b40000,42b40000"
# arr = hexframe_to_array(hexlist)
# print(arr)


if __name__ == "__main__":
    motor1 = MotorSystemIoT()
    signal = [90, 0, -90]
    #stairs_closed(motor1, signal, 1)
    # #print(plant.transfer_function(output='velocity'))
    set_pid(motor1, kp=0.01, ki=0.04, kd=0.0, N=1, beta=1.0)
    sleep(3)

    step_closed(motor1, low_val=0, high_val=100, low_time=0, high_time=2)
    #signal = [30, 40, 50, 60, 70, 80]
    # set_pid(plant, kp = 16.796, ki = 5, kd = 16.441, N = 27.38, beta = 1)
    #step_open(plant, op_point=45, amplitude=5, high_time=200, stab_time=150, uee_time=20)
    # pbrs_open(plant, op_point=45, peak_amp=5, stab_time=150, uee_time=20, divider=35)
    #stair_closed(plant, signal, 10)


