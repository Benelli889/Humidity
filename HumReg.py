#!/u7sr/bin/env python
# coding=utf-8
import Adafruit_DHT
import RPi.GPIO as GPIO
import time
import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont
from smbus import SMBus
import sys, os
sys.path.insert(0, '/home/pi/lib_oled96')
import AbsoluteHumidity
from lib_oled96 import ssd1306
from smbus import SMBus
from PIL import ImageFont
from datetime import datetime 
from datetime import timedelta
from TimerClass import TimerClass
from LoggerClass import LoggerClass
import paho.mqtt.client as mqtt
from threading import Thread

# MQTT
BROKER_ADDRESS = '192.168.178.105'
PORT = 1883
QOS = 0
DATA = "{TEST_DATA}"
TOPIC = 'DHTHum/temp'

client = mqtt.Client()
client.connect(BROKER_ADDRESS, PORT)
client.subscribe(TOPIC, 1)


#Logging Measurements
Log = LoggerClass()
Log.Configure('/home/pi/Humidity/', 'Humidity')

#Timer Heating
CycleSleepTime = 10
Timer = TimerClass()
Timer.SetTimerDuration(timedelta(weeks=0, days=0, seconds=0, microseconds=0, milliseconds=0, minutes=35, hours=0))

# RPi.GPIO Layout verwenden (wie Pin-Nummern)
GPIO.setmode(GPIO.BCM)
GpioHumidity = 24
GPIORelay = 18
GPIO.setup(GPIORelay, GPIO.OUT) 
GPIO.output(GPIORelay, GPIO.LOW)

#Sensortyp und GPIO festlegen
SensorDHT22 = Adafruit_DHT.DHT22

#Luftfeuchtigkeitsschwelle festlegen in %
HumidityThreshold_ON = 78
HumidityThreshold_OFF = 67

# Display einrichten
i2cbus = SMBus(1)            # 0 = Raspberry Pi 1, 1 = Raspberry Pi > 1
oled = ssd1306(i2cbus)

# Ein paar Abkürzungen, um den Code zu entschlacken
draw = oled.canvas

# Schriftarten festlegen
FreeSans14 = ImageFont.truetype('FreeSans.ttf', 15)
FreeSans20 = ImageFont.truetype('FreeSans.ttf', 20)

# Display zum Start löschen
oled.cls()
oled.display()


class MqttHandler(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.client = mqtt.Client()
        self.daemon = True
        self.start()

        self.client.on_connect = self.on_connect
        self.client.on_subscribe = self.on_subscribe
        self.client.on_message = self.on_message
        self.client.on_publish = self.on_publish
        self.client.on_disconnect = self.on_disconnect
        self.client.on_connected_new = self.on_connected_new

        self.client.connect(BROKER_ADDRESS, PORT)
        self.client.subscribe(TOPIC, 1)

        print("Connected to MQTT Broker: " + BROKER_ADDRESS)
        Log.Msg("Connected to MQTT Broker: " + BROKER_ADDRESS)

        self.is_connected_flag = True
        self.ON_MESSAGE_GLOBAL = 0xFF
        self.DATA = 0

    def run(self):
        Log.Msg(" -- RUN --")
        while True:
            self.client.loop()

    def on_connect(self, client, userdata, flags, rc):
        print("connected")
        print("userdata: {}, flags: {}, rc: {}".format(userdata, flags, rc))
        Log.Msg("userdata: {}, flags: {}, rc: {}".format(userdata, flags, rc))

        self.is_connected_flag = True

    def on_subscribe(self, client, userdata, mid, granted_qos):
        print("subscribed")
        Log.Msg("subscribed")

    def on_publish(self, client, userdata, mid):
        print("message published")
        Log.Msg("message published")

    def on_message(self, client, userdata, message):
        print("message printed to topic")
        Log.Msg("message printed to topic")
        print("userdata: {}, message.payload: {}, topic: {} qos: {}".format(
            userdata, str(message.payload), message.topic, message.qos))
        Log.Msg("userdata: {}, --message.payload: {}, --topic: {} qos: {}".format(
            userdata, str(message.payload), message.topic, message.qos))

    def on_disconnect(self, client, userdata, rc):
        print("Client Disconnected")
        Log.Msg("Client Disconnected")

        if rc != 0:
            print("Unexpected disconnection.")
            Log.Msg("Client Disconnected")
            self.is_connected_flag = False

    def on_connected_new(self):

        if (self.is_connected_flag == False):

            try:
                self.client.connect(BROKER_ADDRESS, PORT)

            except:
                print("on_connected_new - connect failure")
                Log.Msg("on_connected_new - connect failure")
                self.is_connected_flag = False

            try:
                self.client.subscribe(TOPIC, 1)
            except:
                print("on_connected_new - subscribe failure")
                Log.Msg("on_connected_new - subscribe failure")
                self.is_connected_flag = False


    def send_data(self, DATA):
        self.DATA = DATA

        (rc, mid) = self.client.publish(TOPIC, DATA, qos=QOS)
        Log.Msg('publish_res: {0} : {1}'.format(rc, mid))

        if (rc != 0):  # mqtt.MQTT_ERR_SUCCESS
            print('publish_res: ', rc, mid)
            Log.Msg('publish_res: {0} : {1}'.format(rc, mid))
            self.is_connected_flag = False




def main_humidity(mqtt):

    # Read Data
    humidity, temperature = Adafruit_DHT.read_retry(SensorDHT22, GpioHumidity)

    # Format for Display
    DTHTemp = '{0:0.1f} C  '.format(temperature)
    DHTHum =  '{0:0.1f}%'.format(humidity)
    TP =      ' - {0:0.1f}'.format(AbsoluteHumidity.TD(humidity,temperature))   
    AbsHum =  '{0:0.1f}mg/m³ '.format(AbsoluteHumidity.AF(humidity,temperature)) 

    HumText = DHTHum + TP + AbsHum 

    if humidity > HumidityThreshold_ON:
        Log.MsgFrequency("Humidity > Threshold: " + "Humidity: {}".format(humidity))
    
        if Timer.State() == Timer.STOPPED:
            Timer.Start()
            print (" Timer started")
            Log.Msg("Timer started: " + "{}".format(Timer.State()))
            GPIO.output(GPIORelay, GPIO.HIGH)

    if Timer.State() == Timer.STARTED:
        if Timer.TimerRunUp():
            print (" Timer run up")
            Log.Msg("Timer run up: " + "{}".format(Timer.State()))
            GPIO.output(GPIORelay, GPIO.LOW)

        if humidity < HumidityThreshold_OFF:
            Timer.Stop()
            GPIO.output(GPIORelay, GPIO.HIGH)
            Log.Msg("Timer Stopped: " + "{}".format(Timer.State()))

    print '------------------'
    print 'Humidity: {:2.2f}'.format(humidity)
    print 'Timer State: ' + '{}'.format(Timer.State())
    print '{:%H:%M:%S}'.format(datetime.now()) 
    print '------------------'
        
    Log.MsgFrequency("Humidity: {:2.2f} Temperature: {:2.2f}".format(humidity, temperature))
    
    DATA = 'Temp":{0:.2f} - "Humidity":{1:.3f}'.format(temperature, humidity)

    mqtt.send_data(DATA)
    
    # Wait
    time.sleep(CycleSleepTime)

    # Ausgaben auf Display schreiben
    oled.cls()
    if Timer.State() == Timer.STARTED:
        draw.text((10,  5), datetime.now().strftime("ON        " + "%H:%M:%S"), font=FreeSans14, fill=1)
    else:
        draw.text((10,  5), datetime.now().strftime("OFF       " + "%H:%M:%S"), font=FreeSans14, fill=1)
    draw.text((10, 25), DTHTemp + DHTHum, font=FreeSans14, fill=1)
    draw.text((10, 45), AbsHum + TP, font=FreeSans14, fill=1)

    oled.display()
    

# Main program logic:
if __name__ == '__main__':

    mqtt = MqttHandler()

    while True:
        main_humidity(mqtt)

        mqtt.on_connected_new()
