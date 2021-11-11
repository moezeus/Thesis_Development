# =========Movement Manual Check Autonomous Truck Trailer MQTT Script========
# Lab ICoDeS, Teknik Fisika, ITB
# Husnul Amri
# ===========================================================================

from ev3dev.ev3 import *
from time import sleep
import paho.mqtt.client as mqtt

# EV3 Related Part
v_ev3 = 0
Qd = 0
m = MediumMotor('outC')  # Steering Motor
mr = LargeMotor('outA')  # Motor for Right Wheel
ml = LargeMotor('outB')  # Motor for Left Wheel
print("Kalibrasi Steering")
m.run_to_abs_pos(position_sp=0, speed_sp=200)
m.stop_action = 'brake'
mr.stop_action = 'brake'
ml.stop_action = 'brake'
sleep(3)
print("posisi awal = " + str(m.position))
m.position = 0

# MQTT Part
speed = 0
steer = 0
broker = "192.168.1.101"
port = 1883
topic = "/EV3_movement/#"


def connect_mqtt() -> mqtt:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt.Client()
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def speed_action(client, userdata, message):
    speed = message.payload.decode("utf-8")
    speed = int(speed)
    # print("speed :" + str(speed))
    mr.run_forever(speed_sp = int(-speed))
    ml.run_forever(speed_sp = int(-speed))


def steer_action(client, userdata, message):
    steer = message.payload.decode("utf-8")
    steer = int(steer)
    print("steer :" + str(steer))
    if steer > 0: #turn left
        m.run_forever(speed_sp=int(30))
    elif steer < 0: #turn right
        m.run_forever(speed_sp=int(-30))
    elif steer == 0: #maintain
        m.run_forever(speed_sp=int(0))

    # angle = m.position * (360 / m.count_per_rot)
    # print("position now:" + str(angle))
    # m.run_to_abs_pos(position_sp = int(steer), speed_sp = 200)


client = connect_mqtt()
client.subscribe(topic)
client.message_callback_add("/EV3_movement/speed_actuate", speed_action)
client.message_callback_add("/EV3_movement/steer_actuate", steer_action)

client.loop_forever()
