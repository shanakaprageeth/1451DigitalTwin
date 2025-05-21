#!/usr/bin/python
__author__ = "shanakaprageeth"

from time import sleep
from switchbot import SwitchBot
import paho.mqtt.client as paho
from paho import mqtt
import argparse
from time import sleep, time
import random
import xml.etree.ElementTree as ET
import sys
import threading
import json
from py_lib_digitaltwin.MQTTClientHandler import MQTTClientHandler
import csv
from datetime import datetime
import requests  # Add this import for making HTTP requests
import yaml

parser = argparse.ArgumentParser(description="Heater Controller")
parser.add_argument("--config", help="Config file path", default="../config.yml")
args = parser.parse_args()

with open(args.config, "r") as config_file:
    config = yaml.safe_load(config_file)

# MQTT Configuration
MQTT_BROKER = config["mqtthost"]
MQTT_PORT = config["mqttport"]
MQTT_USERNAME = config.get("mqttusername",None)
MQTT_PASSWORD = config.get("mqttpassword",None)
MQTT_TLS = config.get("mqtt_tls", False)
SIM_ITERATIONS = config.get("iterations", "inf")

# MQTT Topics
TOPIC_ALL_DATA = config["mqtt_topics"]["all_data"]
TOPIC_HEATER_CONTROL = config["mqtt_topics"]["room_heater_control"]
TOPIC_HEATER_STATE = config["mqtt_topics"]["room_heater_state"]

token = config["switchbot"]["token"]
secret = config["switchbot"]["secret"]
bot_id = config["switchbot"]["bot_id"]
plug_id = config["switchbot"]["plug_id"]

client = None

def turn_on_heater():
    switchbot = SwitchBot(token=token, secret=secret)
    bot = switchbot.device(id=bot_id)
    plug = switchbot.device(id=plug_id)
    plug.command('turn_off')
    sleep(1)
    plug.command('turn_on')
    sleep(1)
    bot.press()
    print("Heater turned on")

def turn_off_heater():
    switchbot = SwitchBot(token=token, secret=secret)
    bot = switchbot.device(id=bot_id)
    plug = switchbot.device(id=plug_id)
    bot.press()
    sleep(1)
    plug.command('turn_off')
    print("Heater turned off")

def my_process_received_message(message):
    payload = str(message.payload.decode("utf-8"))
    print(f"Received topic {message.topic} {payload}")
    
    if message.topic == TOPIC_HEATER_CONTROL:
        try:
            # Parse the payload as JSON
            payload_data = json.loads(payload)
            
            # Check for required fields in the JSON payload
            if (payload_data.get("netSvcType") == 2 and
                payload_data.get("netSvcId") == 1 and
                payload_data.get("msgType") == 1):
                
                # Get the samplingMode and timeout fields
                sampling_mode = payload_data.get("samplingMode")
                timeout = payload_data.get("timeout", 0)  # Default to 0 if not provided
                
                # Turn the heater on or off based on samplingMode
                if sampling_mode == 1:  # Turn on
                    turn_on_heater()
                    state = "on"
                elif sampling_mode == 0:  # Turn off
                    turn_off_heater()
                    state = "off"
                else:
                    print("Invalid samplingMode value in payload")
                    return
                
                # Publish the heater state
                client.publish(TOPIC_HEATER_STATE, get_json_actuator_message(state), qos=1)
                
                # Handle timeout if provided
                if timeout > 0:
                    print(f"Timeout set to {timeout} milliseconds. Heater will turn off after timeout.")
                    sleep(timeout / 1000)  # Convert milliseconds to seconds
                    turn_off_heater()
                    client.publish(TOPIC_HEATER_STATE, get_json_actuator_message("off"), qos=1)
            else:
                print("Invalid payload structure or missing required fields")
        except json.JSONDecodeError:
            print("Failed to parse payload as JSON")

def get_local_time_string():
    """Returns the current local time as a string."""
    from datetime import datetime
    return datetime.now().strftime("%Y-%m-%d_%H:%M:%S")

def get_json_actuator_message(state):
    """Constructs a JSON string with actuator data."""
    from time import time
    timestamp = int(time())  # Get UNIX timestamp (42 bits)
    return f"""{{
        "netSvcType": 2,
        "netSvcId": 1,
        "msgType": 2,
        "msgLength": 0,
        "errorCode": 0,
        "ncapId": 1,
        "timId": 2,
        "channelId": 1,
        "transducersampleData": {1 if state == "on" else 0},
        "timestamp": "{timestamp}0000"
    }}"""

# Global flag to prevent multiple executions of mqtt_test_async
mqtt_test_async_running = False
mqtt_test_async_lock = threading.Lock()  # Lock to prevent race conditions

def heater_controller(
    mqtt_sub_topics=[TOPIC_HEATER_CONTROL],
    mqtt_broker=MQTT_BROKER,
    mqtt_port=MQTT_PORT,
    mqtt_username=MQTT_USERNAME,
    mqtt_password=MQTT_PASSWORD,
    client_id="room_heater",
    enable_tls=MQTT_TLS,
    iterations=SIM_ITERATIONS,
):
    turn_off_heater()
    sleep(20)
    print(f"Starting program")
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
    # subscribe to topics
    print(f"Subscribing to topics {sub_topics}")
    for topic in sub_topics:
        client.subscribe(topic)
    print(f"Starting client loop")
    client.loop_start()
    iterations = float('inf') if "inf" in iterations.lower() else int(iterations)
    i = 0
    while i < iterations:
        i += 1
        print(f"Execution iteration {i}/{iterations}", flush=True)
        try:
            switchbot = SwitchBot(token=token, secret=secret)
            plug = switchbot.device(id=plug_id)
            client.publish(TOPIC_HEATER_STATE, get_json_actuator_message(str(plug.status()['power'])), qos=1)
        except Exception as e:
            print(f"Error: {e}", flush=True)
        sleep(5)
    print("Stopping MQTT client loop", flush=True)
    client.loop_stop()
    client.disconnect()

if __name__ == '__main__':
    heater_controller(
        mqtt_sub_topics=[TOPIC_HEATER_CONTROL],
        mqtt_broker=MQTT_BROKER,
        mqtt_port=MQTT_PORT,
        mqtt_username=MQTT_USERNAME,
        mqtt_password=MQTT_PASSWORD,
        client_id="room_heater",
        enable_tls=MQTT_TLS,
        iterations=SIM_ITERATIONS,
    )

