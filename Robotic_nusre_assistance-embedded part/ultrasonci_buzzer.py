import RPi.GPIO as GPIO
import time
GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)
TRIG_PIN = 5
ECHO_PIN = 6
BUZZER_PIN = 13

minDistance = 50.0


GPIO.setup(TRIG_PIN , GPIO.OUT)
GPIO.setup(ECHO_PIN , GPIO.IN)	
GPIO.setup(BUZZER_PIN , GPIO.OUT)
while(True):
    GPIO.output(TRIG_PIN , False)
    GPIO.output(TRIG_PIN , True)
    time.sleep(10e-6)   #to me in seconds
    GPIO.output(TRIG_PIN , False)
    pulseStart = 0.0
    pulseEnd = 0.0
    
    while(GPIO.input(ECHO_PIN) == 0):
        pulseStart = time.time()
    
    while( GPIO.input(ECHO_PIN) == 1):
        pulseEnd = time.time()
        
    delta = pulseEnd - pulseStart
    Distance = 17150 * delta
    #print(Distance)
    
    if(Distance  <= minDistance  ):
        GPIO.output(BUZZER_PIN , True) 
    else:
        GPIO.output(BUZZER_PIN , False)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    