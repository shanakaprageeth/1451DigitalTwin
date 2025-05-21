#!/usr/bin/python
__author__ = "shanakaprageeth"

import paho.mqtt.client as paho
from paho import mqtt
import argparse
from time import sleep
import random
import xml.etree.ElementTree as ET
import sys
import threading
from py_lib_digitaltwin.MQTTClientHandler import MQTTClientHandler

parser = argparse.ArgumentParser(description="MQTT Subscriber")
parser.add_argument("--mqtt_sub_topics", nargs='+', default=["OUCtest/yourID"], help="MQTT topics to subscribe to")
parser.add_argument("--mqtt_pub_topics", nargs='+', default=["OUCtest/yourID"], help="MQTT topics to publish to")
parser.add_argument("--mqtt_broker", default="broker.hivemq.com", help="MQTT broker address")
parser.add_argument("--mqtt_port", type=int, default=1883, help="MQTT broker port")
parser.add_argument("--mqtt_username", help="MQTT username for authentication")
parser.add_argument("--mqtt_password", help="MQTT password for authentication")
parser.add_argument("--client_id", default="client_1", help="Client ID for the MQTT connection")
parser.add_argument("--enable_tls", action="store_true", default=False, help="Enable TLS for MQTT connection")
parser.add_argument("--iterations", type=str, default="inf", help="Number of iterations for the while loop. Use 'infinity' for an infinite loop.")
args = parser.parse_args()

def my_process_received_message(message):
    payload = str(message.payload.decode("utf-8"))
    print(f"Received topic {message.topic}")
    try:
        ET.fromstring(payload)
        client_id, sensor_id, msg_id, value = process_sensor_message(payload)
        print(f"Received Client {client_id} sensor {sensor_id} msg_id {msg_id} temperature {value}")
    except ET.ParseError:
        print(f"Invalid XML payload: {payload}")
    except Exception as e:
        print(f"Unexpected error while processing payload: {payload}. Error: {e}")

def process_sensor_message(message):
    # Process message in XML format
    root = ET.fromstring(message)
    client_id = root.find("client_id").text
    sensor_id = root.find("sensor_id").text
    msg_id = root.find("msg_id").text
    value = root.find("value").text
    return client_id, sensor_id, msg_id, value

def read_dummy_temp_sensor(client, pub_topics, client_id):
    """Simulates a temperature sensor with a linearly increasing value between 10-30 with noise.
    Publishes the data to the client if available.
    """
    sensor_id = "temp_sensor1"
    if not hasattr(read_dummy_temp_sensor, "current_temp"):
        read_dummy_temp_sensor.current_temp = 10.0
    if not hasattr(read_dummy_temp_sensor, "msg_id"):
        read_dummy_temp_sensor.msg_id = 1
    noise = random.gauss(0, 1)
    read_dummy_temp_sensor.current_temp += 0.5
    read_dummy_temp_sensor.current_temp = min(read_dummy_temp_sensor.current_temp, 30.0)
    temp_value = round(read_dummy_temp_sensor.current_temp + noise, 2)
    msg_id = read_dummy_temp_sensor.msg_id
    for topic in pub_topics:
        pub_message = prepare_sensor_message(client_id, sensor_id, msg_id, temp_value)
        client.publish(topic, f"{pub_message}")
    read_dummy_temp_sensor.msg_id += 1

def prepare_sensor_message(client_id, sensor_id, msg_id, value):
    # Prepare message in proper XML format with header
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<message>
    <client_id>{client_id}</client_id>
    <sensor_id>{sensor_id}</sensor_id>
    <msg_id>{msg_id}</msg_id>
    <value>{value}</value>
</message>"""

# Global flag to prevent multiple executions of mqtt_test_async
mqtt_test_async_running = False
mqtt_test_async_lock = threading.Lock()  # Lock to prevent race conditions

def mqtt_test(
    mqtt_sub_topics=["OUCtest/yourID"],
    mqtt_pub_topics=["OUCtest/yourID"],
    mqtt_broker="broker.hivemq.com",
    mqtt_port=1883,
    mqtt_username=None,
    mqtt_password=None,
    client_id="client_1",
    enable_tls=False,
    iterations="inf",
    process_received_message=None
):
    # setup MQTT client
    print(f"Setting up MQTT client")
    client = paho.Client(client_id=client_id, userdata=None, protocol=paho.MQTTv5)
    if enable_tls:
        client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
    # Set username and password if provided
    if mqtt_username and mqtt_password:
        client.username_pw_set(mqtt_username, mqtt_password)

    handler = MQTTClientHandler(my_process_received_message)
    client.on_connect = handler.on_connect
    client.on_disconnect = handler.on_disconnect
    client.on_publish = handler.on_publish
    client.on_message = handler.on_message
    client.on_subscribe = handler.on_subscribe

    print(f"Connecting to MQTT client {mqtt_broker}:{mqtt_port}")
    # setup the client
    client.connect(mqtt_broker, mqtt_port, 60)
    # Assign subscribe topics
    sub_topics = mqtt_sub_topics if isinstance(mqtt_sub_topics, list) else [mqtt_sub_topics]
    # Assign publisher topics
    pub_topics = mqtt_pub_topics if isinstance(mqtt_pub_topics, list) else [mqtt_pub_topics]
    # subscribe to topics
    print(f"Subscribing to topics {sub_topics}")
    for topic in sub_topics:
        client.subscribe(topic)
    print(f"Starting client loop")
    client.loop_start()

    iterations = float('inf') if "inf" in args.iterations.lower() else int(args.iterations)
    i = 0
    while i < iterations:
        i += 1
        print(f"Execution iteration {i}/{iterations}", flush=True)
        read_dummy_temp_sensor(client, pub_topics, client_id)
        sleep(5)

    print("Stopping MQTT client loop", flush=True)
    client.loop_stop()
    client.disconnect()

if __name__ == '__main__':
    mqtt_test(
        mqtt_sub_topics=args.mqtt_sub_topics,
        mqtt_pub_topics=args.mqtt_pub_topics,
        mqtt_broker=args.mqtt_broker,
        mqtt_port=args.mqtt_port,
        mqtt_username=args.mqtt_username,
        mqtt_password=args.mqtt_password,
        client_id=args.client_id,
        enable_tls=args.enable_tls,
        iterations=args.iterations
    )
