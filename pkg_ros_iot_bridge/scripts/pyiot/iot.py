from multiprocessing.dummy import Pool
import time
import requests

import sys
import paho.mqtt.client as mqtt #import the client1
import time




class print_colour:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


# -----------------  MQTT SUB -------------------
def iot_func_callback_sub(client, userdata, message):
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)

def mqtt_subscribe_thread_start(arg_callback_func, arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_qos):
    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_message = arg_callback_func
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.subscribe(arg_mqtt_topic, arg_mqtt_qos)
        time.sleep(1) # wait
        # mqtt_client.loop_forever() # starts a blocking infinite loop
        mqtt_client.loop_start()    # starts a new thread
        return 0
    except:
        return -1

def http_send_spread(id, final_x, final_y, final_theta):
    url = "https://script.google.com/macros/s/AKfycbzOo4pSZ-JHofkbW7-hDTQRib6OJ-DpYTnfmO_uszuqhVH1pMg/exec"


    url2 = "https://script.google.com/macros/s/AKfycbygyuHa3Z-ObngJexz4yhCXILVuZ_7nUnNoK8rLuZapLCiMYbPI/exec" # isko yaha place krna h 
    #https://script.google.com/macros/s/AKfycbw850dk4moVgebU2GGe0PUQUvvg8jTpSjBQCawJt3_13vgujLk/exec   # ye unki id h 88888888888888888888888

    parameter = {"id":id, "final_x":final_x, "final_y":final_y, "final_theta":final_theta}
    parameter_eyrc = {"id":"task1", "team_id":"VB_0528", "unique_id":"JaiMataD", "turtle_x":final_x, "turtle_y":final_y, "turtle_theta":final_theta}
    response = requests.get(url, params = parameter)
    response2 = requests.get(url2, params = parameter_eyrc)



# -----------------  MQTT PUB -------------------
def mqtt_publish(arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos, arg_spreadsheet_id):
    try:
        mqtt_client = mqtt.Client("mqtt_pub")
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.loop_start()

        print("Publishing message to topic", arg_mqtt_topic)
        mqtt_client.publish(arg_mqtt_topic, str(arg_mqtt_message), arg_mqtt_qos)
        http_parameters = {"id":arg_spreadsheet_id, "final_x": arg_mqtt_message[0], "final_y":arg_mqtt_message[1], "final_theta":arg_mqtt_message[2]}
        http_send_spread(**http_parameters)
        time.sleep(2)

        time.sleep(0.1) # wait

        mqtt_client.loop_stop() #stop the loop
        return 0
    except:
        return -1
