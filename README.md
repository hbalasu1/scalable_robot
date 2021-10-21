# scalable_robot
Scalable Autonomous Navigation Robot with OpenVINO

Preparation on Linux PC Hosting OpenVINO Runtime and OpenCV

Make sure MQTT Broker is running on Linux Host PC (that is **running mqtt_traffic_recognition.py**)
To test the broker, you could run this command to see if the GPIO can be toggled manually likewise

**Manual MQTT Publish Command:**

Type '**mosquitto_pub -h localhost -t inTopic -m "0"**' to toggle GPIOs for Robot to go straight

or
Type '**mosquitto_pub -h localhost -t inTopic -m "1"**' to toggle GPIOs for Robot to turn left. 


for **Python MQTT to work**, please install paho-mqtt with the command below. 
sudo apt-get install python3-paho-mqtt

**Edit** mqtt_traffic_recognition.py

**1. Assign the right MQTT broker IP address**
broker="192.168.0.103" 
#broker="localhost" <=== **Please "localhost" as MQTT broker if no external MQTT Broker**

**2. Assign the correct RTSP IP Address**
camera = cv2.VideoCapture('rtsp://192.168.0.121:8554/mjpeg/1')

**Run mqtt_traffic_recognition.py and observe the changes in motor/gpio direction.**


For 320x240 Resolution & MQTT Broken Address
**For 320x240 Resolution** 
Go to Micro-RSTP library and edit OV2640.cpp (in Notepad++/VSCode
Change all **framesize_VGA** to **framesize_QVGA**

**For MQTT Broker IP Address**
Go to file scalable_robot/ESP32-RTSP-mqtt/src/main.cpp
Edit **const char* mqtt_server = "192.168.0.103"**; to align to Linux MQTT Broker Host IP Address

**Recompile Arduino INO and reflash ESP32-CAM Board**
