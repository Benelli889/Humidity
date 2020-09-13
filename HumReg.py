#!/u7sr/bin/env python
# coding=utf-8
from datetime import datetime
from LoggerClass import LoggerClass
from TimerClass import TimerClass
from datetime import timedelta
from PIL import ImageFont
from lib_oled96 import ssd1306
import AbsoluteHumidity
import Adafruit_DHT
import RPi.GPIO as GPIO
import time
import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont
from smbus import SMBus
import sys
import os
sys.path.insert(0, '/home/pi/lib_oled96')

# Logging Measurements
Log = LoggerClass()
Log.Configure('/home/pi/Humidity/', 'Humidity')

# Timer Heating
CycleSleepTime = 10
Timer = TimerClass()
Timer.SetTimerDuration(timedelta(weeks=0, days=0, seconds=0,
                                 microseconds=0, milliseconds=0, minutes=35, hours=0))

# RPi.GPIO Layout verwenden (wie Pin-Nummern)
GPIO.setmode(GPIO.BCM)
GpioHumidity = 24
GPIORelay = 18
GPIO.setup(GPIORelay, GPIO.OUT)
GPIO.output(GPIORelay, GPIO.LOW)

# Sensortyp und GPIO festlegen
SensorDHT22 = Adafruit_DHT.DHT22

# Luftfeuchtigkeitsschwelle festlegen in %
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


while True:
    # Read Data
    humidity, temperature = Adafruit_DHT.read_retry(SensorDHT22, GpioHumidity)

    # Format for Display
    DTHTemp = '{0:0.1f} C  '.format(temperature)
    DHTHum = '{0:0.1f}%'.format(humidity)
    TP = ' - {0:0.1f}'.format(AbsoluteHumidity.TD(humidity, temperature))
    AbsHum = '{0:0.1f}mg/m² '.format(
        AbsoluteHumidity.AF(humidity, temperature))

    HumText = DHTHum + TP + AbsHum

    if humidity > HumidityThreshold_ON:
        Log.MsgFrequency("Humidity > Threshold: " +
                         "Humidity: {}".format(humidity))

        if Timer.State() == Timer.STOPPED:
            Timer.Start()
            print(" Timer started")
            Log.Msg("Timer started: " + "{}".format(Timer.State()))
            GPIO.output(GPIORelay, GPIO.HIGH)

    if Timer.State() == Timer.STARTED:
        if Timer.TimerRunUp():
            print(" Timer run up")
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

    Log.MsgFrequency(
        "Humidity: {:2.2f} Temperature: {:2.2f}".format(humidity, temperature))

    # Wait
    time.sleep(CycleSleepTime)

    # Ausgaben auf Display schreiben
    oled.cls()
    if Timer.State() == Timer.STARTED:
        draw.text((10,  5), datetime.now().strftime(
            "ON        " + "%H:%M:%S"), font=FreeSans14, fill=1)
    else:
        draw.text((10,  5), datetime.now().strftime(
            "OFF       " + "%H:%M:%S"), font=FreeSans14, fill=1)
    draw.text((10, 25), DTHTemp + DHTHum, font=FreeSans14, fill=1)
    draw.text((10, 45), AbsHum + TP, font=FreeSans14, fill=1)

    oled.display()
