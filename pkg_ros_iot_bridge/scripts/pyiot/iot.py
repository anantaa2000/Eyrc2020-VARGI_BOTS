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

def http_send_spread_inventory(data):
    url = "https://script.google.com/macros/s/AKfycbxQ8X4Lu1aTrL4uoiuZ1WNIB91gNss29FE7IZhiV3U-AQjPlWI/exec"
    url2 = "https://script.google.com/macros/s/AKfycbw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7/exec"
    

    parameter = {"id":"Inventory", "Team Id":"VB_0528", "Unique Id":"JaiMataD", "SKU":data[1], "Item":data[2], "Priority":data[3], "Storage Number":data[4], "Cost":data[5], "Quantity":data[6]}
    response = requests.get(url, params = parameter)
    response2 = requests.get(url2, params = parameter)
    print response
    print response2

def http_send_spread_incoming_orders(data):
    url = "https://script.google.com/macros/s/AKfycbxQ8X4Lu1aTrL4uoiuZ1WNIB91gNss29FE7IZhiV3U-AQjPlWI/exec"
    url2 = "https://script.google.com/macros/s/AKfycbw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7/exec"

    parameter = {"id":"IncomingOrders", "Team Id":"VB_0528", "Unique Id":"JaiMataD", "Order ID":data[1],"Order Date and Time":data[2], "Item":data[3], "Priority":data[4], "Order Quantity":data[5], "City":data[6], "Longitude":data[7], "Latitude":data[8], "Cost":data[9]}
    response = requests.get(url, params = parameter)
    response2 = requests.get(url2, params = parameter)
    print response
    print response2


def http_send_spread_orders_dispatched(data):
    url = "https://script.google.com/macros/s/AKfycbxQ8X4Lu1aTrL4uoiuZ1WNIB91gNss29FE7IZhiV3U-AQjPlWI/exec"
    url2 = "https://script.google.com/macros/s/AKfycbw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7/exec"

    parameter = {"id":"OrdersDispatched", "Team Id":"VB_0528", "Unique Id":"JaiMataD", "Order ID":data[1], "City":data[2], "Item":data[3], "Priority":data[4], "Dispatch Quantity":data[5], "Cost":data[6],"Dispatch Status":data[7], "Dispatch Date and Time":data[8]}
    response = requests.get(url, params = parameter)
    response2 = requests.get(url2, params = parameter)
    print response
    print response2


def http_send_spread_orders_shipped(data):
    url = "https://script.google.com/macros/s/AKfycbxQ8X4Lu1aTrL4uoiuZ1WNIB91gNss29FE7IZhiV3U-AQjPlWI/exec"
    url2 = "https://script.google.com/macros/s/AKfycbw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7/exec"

    parameter = {"id":"OrderShipped", "Team Id":"VB_0528", "Unique Id":"JaiMataD", "Order ID":data[1], "City":data[2], "Item":data[3], "Priority":data[4], "Shipped Quantity":data[5], "Cost":data[6],"Shipped Status":data[7], "Shipped Date and Time":data[8], "Estimated Time of Delivery":data[9]}
    response = requests.get(url, params = parameter)
    # response2 = requests.get(url2, params = parameter)
    print response
    # print response2



# -----------------  MQTT PUB -------------------
def mqtt_publish(arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos, arg_spreadsheet_id):
    try:
        mqtt_client = mqtt.Client("mqtt_pub")
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.loop_start()
        print("Publishing message to topic", arg_mqtt_topic)
        mqtt_client.publish(arg_mqtt_topic, str(arg_mqtt_message), arg_mqtt_qos)


        if arg_mqtt_message[0]=="Inventory":
            http_send_spread_inventory(arg_mqtt_message)

        if arg_mqtt_message[0]=="IncomingOrders":
            http_send_spread_incoming_orders(arg_mqtt_message)

        if arg_mqtt_message[0]=="OrdersDispatched":
            http_send_spread_orders_dispatched(arg_mqtt_message)

        if arg_mqtt_message[0]=="OrderShipped":
            http_send_spread_orders_shipped(arg_mqtt_message)


        
        time.sleep(2)

        mqtt_client.loop_stop() #stop the loop
        return 0
    except:
        return -1
