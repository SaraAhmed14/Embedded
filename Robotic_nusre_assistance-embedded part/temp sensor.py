#import modules
import time
import RPi.GPIO as GPIO
#GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

try:
    while 1:
        tempStore = open("/sys/bus/w1/devices/28-00000d96266a/w1_slave")	#change this number to the Device ID of your sensor
        data = tempStore.read()
        tempStore.close()
        tempData = data.split("\n")[1].split(" ")[9]
        temperature = float(tempData[2:])
        temperature = temperature/1000
        print (temperature)

except KeyboardInterrupt:
    GPIO.cleanup()
    print ("Program Exited Cleanly")