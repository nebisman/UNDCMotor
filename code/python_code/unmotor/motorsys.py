# Required libraries


import paho.mqtt.client as mqtt
import control as ct
import struct
from queue import Queue
from math import ceil
import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.signal import cont2discrete
import csv
from scipy.interpolate import PchipInterpolator



BROKER = "18.204.70.207"
PORT = 1883
USER = "user"
PASSWORD = "user"
PLANT_NUMBER = "2201"


#default configuration

PATH_DEFAULT = r"./experiment_files/"
PATH_DATA = str(Path(__file__).parent) + r"/datafiles/"
FONT_SIZE = 12
Path(PATH_DEFAULT).mkdir(exist_ok=True)




""" This is the class defining the IoT motor system"""
class MotorSystemIoT:

    def __init__(self, broker_address=BROKER, port=PORT, plant_number=PLANT_NUMBER, user=USER, password=PASSWORD,
                 client_id="", clean_session=True):

        codes = {"SYS_USER_SIGNALS_CLOSED": "/motor/motor_" + str(plant_number) + "/user/sig_closed",
                 "SYS_USER_SIGNALS_OPEN": "/motor/motor_" + str(plant_number) + "/user/sig_open",
                 "USER_SYS_SET_REF": "/motor/user/motor_" + str(plant_number) + "/set_ref",
                 "USER_SYS_SET_PID": "/motor/user/motor_" + str(plant_number) + "/set_pid",
                 "USER_SYS_STEP_CLOSED": "/motor/user/motor_" + str(plant_number) + "/step_closed",
                 "USER_SYS_STAIRS_CLOSED": "/motor/user/motor_" + str(plant_number) + "/stairs_closed",
                 "USER_SYS_PRBS_OPEN": "/motor/user/motor_" + str(plant_number) + "/prbs_open",
                 "USER_SYS_STEP_OPEN": "/motor/user/motor_" + str(plant_number) + "/step_open",
                 "USER_SYS_SET_GENCON": "/motor/user/motor_" + str(plant_number) + "/set_gencon",
                 "USER_SYS_PROFILE_CLOSED": "/motor/user/motor_" + str(plant_number) + "/prof_closed",
                 "MOTOR_SAMPLING_TIME": 0.02,
                 "BUFFER_SIZE": 25,
                 }
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
        return

    def on_message(self, client, userdata, message):
        return


    def on_subscribe(self, client, userdata, mid, granted_qos):
        print("Subscribed: ", mid, " ", granted_qos)
        return


    def on_publish(self, client, userdata, mid):
        pass



    def connect(self):
        self.client.username_pw_set(USER, PASSWORD)
        self.client.connect(self.broker_address, self.port)
        self.client.loop_start()

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()

    def subscribe(self, topic, qos=2):
        self.client.subscribe(topic, qos)

    def publish(self, topic, message, qos=2):
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

def read_csv_file(filepath= PATH_DATA + 'DCmotor_static_gain_response.csv'):
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

if __name__ == "__main__":
    motor1 = MotorSystemIoT(plant_number=2201)
    print(motor1.speed_from_volts(2.5))




