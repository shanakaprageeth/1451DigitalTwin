/** 
* @author W. A. Shanaka P. Abeysiriwardhana
* @copyright shanakaprageeth
* @version 0.0.1
* @date 2025-03-30
* @brief This is a demo program for M5Stack Core2
* @details This program is a demo for M5Stack Core2 and NCAP. It reads data from SHT4X sensor and publish it to MQTT server.
* @platform: m5stack core
* @sensor: SHT4X series sensor
**/

#include <M5Stack.h>
#include <M5UnitENV.h>
#include <WiFi.h>
#include <PubSubClient.h>
# include "time.h"

// constants
// loop delay
#define LOW_POWER_MODE false
#if LOW_POWER_MODE
const int loop_delay = 60000; // 60 seconds
const bool low_power_mode = true; // low power mode
#else
const int loop_delay = 20000; // 10 seconds
const bool low_power_mode = false; // low power mode
#endif

// device name
const char* device_name = "core_2";
// Wi-Fi settings
const char* ssid = "GL-AR750-310";
const char* password = "goodlife"; 
// mqtt settings
const char* mqtt_server = "192.168.8.101";
const int mqtt_port = 1883;
//String mqtt_topic_name = "_1451DT/" + (String)device_name + "/sensor/data";
String mqtt_topic_name = "_1451DT/" + (String)device_name + "/sensor/data";
// setup ntp and timezone
const char* ntpServer = "ntp.nict.jp";
const long  gmtOffset_sec = 3600 * 9;
const int   daylightOffset_sec = 0;

// global variables
struct tm timeinfo;
// handle sensor
SHT4X sht4;
BMP280 bmp;
// network
String ip_address;
//mqtt
WiFiClient esp_client;
PubSubClient mqttclient(esp_client);
// to write formatted messages
String formatted_mqtt_msg = "";
bool mqtt_publish_status = false;

void setupWifi(){
    Serial.println("starting wifi setup");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    ip_address = WiFi.localIP().toString();
    String formatted_serial_msg = "\ncompleted wifi setup\nSSID: " + String(ssid) + "\nLocal IP: " + ip_address;
    Serial.println(formatted_serial_msg);
}

void setupSensor(){
    Serial.println("starting SHT4X sensor setup");
    if (!sht4.begin(&Wire, SHT40_I2C_ADDR_44, 21, 22, 400000U)) {
        Serial.println("Couldn't find SHT4x");
        while (1) delay(1);
    }

    // You can have 3 different precisions, higher precision takes longer
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);

    if (!bmp.begin(&Wire, BMP280_I2C_ADDR, 21, 22, 400000U)) {
        Serial.println("Couldn't find BMP280");
        while (1) delay(1);
    }
    /* Default settings from datasheet. */
    bmp.setSampling(BMP280::MODE_NORMAL,     /* Operating Mode. */
                    BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    BMP280::FILTER_X16,      /* Filtering. */
                    BMP280::STANDBY_MS_500); /* Standby time. */
    Serial.println("completed SHT4X sensor setup");
}

void setupDisplay(){
    M5.Lcd.fillScreen(WHITE);
    delay(500);
    M5.Lcd.fillScreen(RED);
    delay(500);
    M5.Lcd.fillScreen(GREEN);
    delay(500);
    M5.Lcd.fillScreen(BLUE);
    delay(500);
    M5.Lcd.fillScreen(BLACK);
    delay(500);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("Device: %s \n  ssid: %s\n  IP:%s", device_name, ssid, ip_address.c_str());
}

void setup() {
    M5.begin();
    M5.Power.begin();
    Serial.begin(115200);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    setupSensor();
    setupWifi();
    mqttclient.setServer(mqtt_server, mqtt_port);
    mqttclient.setCallback(mqttCallback);
    if (mqttclient.connect(ip_address.c_str())) {
        mqttclient.subscribe(mqtt_topic_name.c_str());
        Serial.print("Subscribed to topic: ");
        Serial.println(mqtt_topic_name);
    }
    if (low_power_mode){
        M5.Lcd.writecommand(ILI9341_DISPOFF);
        M5.Lcd.setBrightness(0);
    }
    else{
        setupDisplay();
    }  
}

// Function to read sensor data
// Returns true if successful, false otherwise
// This function reads data from the SHT4X and BMP280 sensors
// sht4.cTemp is the temperature in Celsius
// sht4.humidity is the humidity in percentage
// bmp.cTemp is the temperature in Celsius
// bmp.pressure is the pressure in Pascal
// bmp.altitude is the altitude in meters
bool readSensor(){
    bool status = true;
    if (!sht4.update()){
        status = false;
    }
    if (!bmp.update()){
        status = false;
    }
    return status;
}

String getLocalTimeString(){
    getLocalTime(&timeinfo);
    return String(timeinfo.tm_year + 1900) + "_" + String(timeinfo.tm_mon + 1) +
    "_" + String(timeinfo.tm_mday + 1) + "_" +
     String(timeinfo.tm_hour) + "_" + String(timeinfo.tm_min) + 
     "_" + String(timeinfo.tm_sec);
}

String generateFormattedMessage(){
    return "Device: " +  String(device_name) +
        "\n SSID:   " + String(ssid) +
        "\n IP:     " + String(ip_address) + 
        "\n MQTT:   " + String(mqtt_server) + ":"+ String(mqtt_port) +
        "\n  TOPIC: " + String(mqtt_topic_name) +
        "\n  CONN:  " + String(mqttclient.connected()) +
        "\n LocalTime:" + getLocalTimeString() +
        "\n TempSHT: " + String(sht4.cTemp) + 
        "\n Humidity:" + String(sht4.humidity) +
        "\n Pressure:" + String(bmp.pressure) +
        "\n Altitude:" + String(bmp.altitude) +
        "\n TempBMP: "+ String(bmp.cTemp);
}



String generateMQTTMessageJSON(const std::vector<int>& channelIds) {
    String json_message = "{ \"netSvcType\": 2," +
                          "\"netSvcId\": 3," +
                          "\"msgType\": 2," +
                          "\"msgLength\": 0," +
                          "\"errorCode\": 0," +
                          "\"ncapId\": 1," +
                          "\"timId\": 1," +
                          "\"channelIds\": [";

    // Add channel IDs to the JSON
    for (size_t i = 0; i < channelIds.size(); ++i) {
        json_message += String(channelIds[i]);
        if (i < channelIds.size() - 1) {
            json_message += ", ";
        }
    }
    json_message += "],";

    // Add transducer sample data based on channel IDs
    json_message += "\"transducerSampleDatas\": [";
    for (size_t i = 0; i < channelIds.size(); ++i) {
        switch (channelIds[i]) {
            case 0:
                json_message += String(sht4.cTemp); // Temperature from SHT4
                break;
            case 1:
                json_message += String(bmp.cTemp); // Temperature from BMP280
                break;
            case 2:
                json_message += String(sht4.humidity); // Humidity from SHT4
                break;
            case 3:
                json_message += String(bmp.pressure); // Pressure from BMP280
                break;
            case 4:
                json_message += String(bmp.altitude); // Altitude from BMP280
                break;
            default:
                json_message += "null"; // Invalid channel ID
                break;
        }
        if (i < channelIds.size() - 1) {
            json_message += ", ";
        }
    }
    json_message += "],";

    // Add timestamp
    json_message += "\"timestamp\": \"" + String((uint32_t)time(nullptr)) + "0000\"";

    json_message += "}";
    return json_message;
}

String generateMQTTMessageXML(const std::vector<int>& channelIds) {
    String xml_message = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
    xml_message += "<TEDS>";
    xml_message += "<netSvcType>2</netSvcType>";
    xml_message += "<netSvcId>3</netSvcId>";
    xml_message += "<msgType>2</msgType>";
    xml_message += "<msgLength>0</msgLength>";
    xml_message += "<errorCode>0</errorCode>";
    xml_message += "<ncapId>1</ncapId>";
    xml_message += "<timId>1</timId>";

    // Add channel IDs as a comma-separated list
    xml_message += "<channelIds>";
    for (size_t i = 0; i < channelIds.size(); ++i) {
        xml_message += String(channelIds[i]);
        if (i < channelIds.size() - 1) {
            xml_message += ",";
        }
    }
    xml_message += "</channelIds>";

    // Add transducer sample data as a comma-separated list
    xml_message += "<transducerSampleDatas>";
    for (size_t i = 0; i < channelIds.size(); ++i) {
        switch (channelIds[i]) {
            case 0:
                xml_message += String(sht4.cTemp); // Temperature from SHT4
                break;
            case 1:
                xml_message += String(bmp.cTemp); // Temperature from BMP280
                break;
            case 2:
                xml_message += String(sht4.humidity); // Humidity from SHT4
                break;
            case 3:
                xml_message += String(bmp.pressure); // Pressure from BMP280
                break;
            case 4:
                xml_message += String(bmp.altitude); // Altitude from BMP280
                break;
            default:
                xml_message += "null"; // Invalid channel ID
                break;
        }
        if (i < channelIds.size() - 1) {
            xml_message += ",";
        }
    }
    xml_message += "</transducerSampleDatas>";

    // Add timestamp
    xml_message += "<timestamp>" + String((uint32_t)time(nullptr)) + "0000</timestamp>";

    xml_message += "</TEDS>";
    return xml_message;
}

bool publishDataSerial(){
    Serial.println(generateFormattedMessage());
    return true;
}

bool publishDataDisplay(){
    M5.Lcd.setTextSize(2.0);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.printf(generateFormattedMessage().c_str());
    return true;
}

bool publishDataMQTT(std::vector<int>& channelIds){
    if (mqttclient.connected()) {
        const char* mqtt_message = generateMQTTMessageXML(channelIds).c_str();
        Serial.print(mqtt_message);
        mqttclient.publish(mqtt_topic_name.c_str(), mqtt_message);
        //mqttclient.publish(mqtt_topic_name.c_str(), generateMQTTMessageJSON(channelIds).c_str());
        return true;
    }
    return false;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.print("Message received on topic: ");
    Serial.println(topic);
    Serial.print("Message: ");
    Serial.println(message);

    // Parse the message to check for required fields
    bool isValidMessage = false;
    std::vector<int> channelIds = {0, 1, 2, 3}; // Default channel IDs

    // Check if the message is JSON
    if (message.indexOf("\"netSvcType\": 2") != -1 &&
        message.indexOf("\"netSvcId\": 3") != -1 &&
        message.indexOf("\"msgType\": 1") != -1) {
        isValidMessage = true;

        // Check if 'channelIds' is provided in the JSON message
        int channelIdsStart = message.indexOf("\"channelIds\": [");
        if (channelIdsStart != -1) {
            channelIds.clear(); // Clear the default vector
            int channelIdsEnd = message.indexOf("]", channelIdsStart);
            String channelIdsStr = message.substring(channelIdsStart + 14, channelIdsEnd);
            channelIdsStr.trim();

            // Parse channel IDs from the string
            while (channelIdsStr.length() > 0) {
                int commaIndex = channelIdsStr.indexOf(",");
                if (commaIndex == -1) {
                    channelIds.push_back(channelIdsStr.toInt());
                    break;
                } else {
                    channelIds.push_back(channelIdsStr.substring(0, commaIndex).toInt());
                    channelIdsStr = channelIdsStr.substring(commaIndex + 1);
                    channelIdsStr.trim();
                }
            }
        }
    }
    // Check if the message is XML
    else if (message.indexOf("<netSvcType>2</netSvcType>") != -1 &&
             message.indexOf("<netSvcId>3</netSvcId>") != -1 &&
             message.indexOf("<msgType>1</msgType>") != -1) {
        isValidMessage = true;

        // Check if 'channelIds' is provided in the XML message
        int channelIdsStart = message.indexOf("<channelIds>");
        if (channelIdsStart != -1) {
            channelIds.clear(); // Clear the default vector
            int channelIdsEnd = message.indexOf("</channelIds>", channelIdsStart);
            String channelIdsStr = message.substring(channelIdsStart + 12, channelIdsEnd);
            channelIdsStr.trim();

            // Parse channel IDs from the XML string
            while (channelIdsStr.length() > 0) {
                int commaIndex = channelIdsStr.indexOf(",");
                if (commaIndex == -1) {
                    channelIds.push_back(channelIdsStr.toInt());
                    break;
                } else {
                    channelIds.push_back(channelIdsStr.substring(0, commaIndex).toInt());
                    channelIdsStr = channelIdsStr.substring(commaIndex + 1);
                    channelIdsStr.trim();
                }
            }
        }
    }

    if (isValidMessage) {
        Serial.println("Valid message detected, publishing data...");
        readSensor();
        publishDataMQTT(channelIds);
    } else {
        Serial.println("Invalid message received, ignoring...");
    }
}


void loop() {
    static unsigned long last_display_update = 0; // Track the last display update time
    unsigned long current_time = millis();
    if (!mqttclient.connected()) {
        mqtt_publish_status = false;
        if (mqttclient.connect(ip_address.c_str())) {
            mqttclient.subscribe(mqtt_topic_name.c_str()); // Re-subscribe if disconnected
        }
    }
    mqttclient.loop(); // Process incoming MQTT messages

    if (readSensor()) {
        if (!mqttclient.connected()) {
            mqtt_publish_status = false;
            mqttclient.connect(ip_address.c_str());
        }
        std::vector<int> channelIds = {0, 1, 2, 3};
        if (publishDataMQTT(channelIds)) {
            mqtt_publish_status = true;
        } else {
            mqtt_publish_status = false;
        }
        // Update display only if 1 minute has passed
        if (low_power_mode){
        }
        else{
            publishDataSerial();
            if (current_time - last_display_update >= 60000) {
                publishDataDisplay();
                last_display_update = current_time;
            }
        }
        mqtt_publish_status = false;
    }
    else {
        Serial.println("Failed to read sensor data");
        mqtt_publish_status = false;
    }
    delay(loop_delay);
}