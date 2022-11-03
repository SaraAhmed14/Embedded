import cv2
import numpy as np
import pyzbar.pyzbar as pyz
import time
import threading
from turtle import right
from xmlrpc.client import boolean
import RPi.GPIO as GPIO
import face_recognition
#import ultrasonci_buzzer.py


def rescale_frame(frame, percent):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)

def showframe(frame):
    cv2.imshow("Searching ....", rescale_frame(frame,percent=75))
    key=cv2.waitKey(1)
    if key == 0:
        exit
    
def searching(d):
    print("\n\n\n Now Codee is Looking For Door Number : "+ d)
    d=str.encode(d)
    cap=cv2.VideoCapture(0)
    f=cv2.FONT_HERSHEY_PLAIN
    while True:
        _,frame = cap.read()
        showframe(frame)
        decodeobjects = pyz.decode(frame)
        for obj in decodeobjects:
            cv2.putText(frame, str(obj.data).replace('b','Door Number '), (50, 50), f, 2,(255, 0, 0), 3)
            if d == obj.data:
                print(" I Reach The Distnation")
                cv2.destroyAllWindows()
                return 1
                #exit()
            else:
                output_msg = d
                cv2.destroyAllWindows()
                return 0




########################################
#rana code
            
# This is a demo of running face recognition on live video from your webcam. It's a little more complicated than the
# other example, but it includes some basic performance tweaks to make things run a lot faster:
#   1. Process each video frame at 1/4 resolution (though still display it at full resolution)
#   2. Only detect faces in every other frame of video.

# PLEASE NOTE: This example requires OpenCV (the `cv2` library) to be installed only to read from your webcam.
# OpenCV is *not* required to use the face_recognition library. It's only required if you want to run this
# specific demo. If you have trouble installing it, try any of the other demos that don't require it instead.

# Get a reference to webcam #0 (the default one)
percent = 75
start_time = time.perf_counter()
def face(truename):
    video_capture = cv2.VideoCapture(0)
    '''
    video_capture.set(3,3000)
    video_capture.set(4,3000)
    '''
    # Load a sample picture and learn how to recognize it.
    print("I reached here")
   

    # Load a second sample picture and learn how to recognize it.

    rana_image = face_recognition.load_image_file("rana1.jpeg")
    rana_face_encoding = face_recognition.face_encodings(rana_image)[0]
    youssef_image = face_recognition.load_image_file("yelzahar.jpeg")
    youssef_face_encoding = face_recognition.face_encodings(youssef_image)[0]
    seif_image = face_recognition.load_image_file("seif.jpeg")
    seif_encoding = face_recognition.face_encodings(seif_image)[0]
    saralaa2_image = face_recognition.load_image_file("saraalaa2.jpeg")
    saralaa_encoding2 = face_recognition.face_encodings(saralaa2_image)[0]
    dr_muhamed_image = face_recognition.load_image_file("drmuhamrd.jpg")
    dr_muhamed_encoding = face_recognition.face_encodings(dr_muhamed_image)[0]
    shady_image = face_recognition.load_image_file("shady.jpeg")
    shady_encoding = face_recognition.face_encodings(shady_image)[0]
    
    
    # Create arrays of known face encodings and their names
    known_face_encodings = [
        rana_face_encoding,
        youssef_face_encoding,
        seif_encoding,
        saralaa_encoding2,
        dr_muhamed_encoding,
        shady_encoding
        
    ]
    known_face_names = [
        "Rana Khaled",
        "youssef",
        "Seif",
        "Sara Alaa",
        "Dr Mohamed",
        "shady"
    
    ]
    print("I reached here")
    # Initialize some variables
    face_locations = []
    face_encodings = []
    face_names = []
    process_this_frame = True
    classnames=[]
    print("I reached here")
    
    while True:
        timenow = time.perf_counter()
        flag=0
        # Grab a single frame of video
        ret, frame = video_capture.read()
        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(frame,(0,0), fx=0.25, fy=0.25)
        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = small_frame[:, :, ::-1]

        # Only process every other frame of video to save time
        if process_this_frame:
            # Find all the faces and face encodings in the current frame of video
            face_locations = face_recognition.face_locations(rgb_small_frame)
            face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

            face_names = []
            for face_encoding in face_encodings:
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                faceDis = face_recognition.face_distance(known_face_encodings, face_encoding)
                print(faceDis,matches)
                name = "Unknown"
                match = np.argmin(faceDis)

                # # If a match was found in known_face_encodings, just use the first one.
                if True in matches:
                    first_match_index = matches.index(True)
                    name = known_face_names[first_match_index]
                    print(timenow)
                    if(timenow-start_time>30):
                        if name == truename:
                            video_capture.release() 
                            cv2.destroyAllWindows()
                            return 1
                        elif name!=truename:
                            video_capture.release()
                            cv2.destroyAllWindows()
                            return 0
                # Or instead, use the known face with the smallest distance to the new face
                face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                best_match_index = np.argmin(face_distances)
                if matches[best_match_index]:
                    name = known_face_names[best_match_index]
                
                face_names.append(name) 

                """face_names.append(name)
                if flag==1:
                    time.sleep(5)
                    print(flag)
                    cv2.destroyAllWindows()
                else:
                    time.sleep(5)
                    print(flag)
                    cv2.destroyAllWindows()
                """  
       
        process_this_frame = not process_this_frame


        # Display the results
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4

            # Draw a box around the face
            cv2.rectangle(frame, (left, top), (right, bottom), (67, 23, 255), 2)

            # Draw a label with a name below the face
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        # Display the resulting image
        cv2.imshow('Video', frame)

        # Hit 'q' on the keyboard to quit!
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release handle to the webcam
    video_capture.release()
    cv2.destroyAllWindows()
    print("I reached here")

  
    




################################




r1 = 17
r2 = 18
l1 = 15
l2 = 14
medicine_pin = 16
# set GPIO pins as inputs
leftSensor = 20
rightSensor = 21
TRIG_PIN = 5
ECHO_PIN = 6
BUZZER_PIN = 13
minDistance = 50.0

# set pin mapping to BOARD
GPIO.setmode(GPIO.BCM)


# turn off channel warnings messages
GPIO.setwarnings(False)


#motor
GPIO.setup(r1, GPIO.OUT)
GPIO.setup(r2, GPIO.OUT)
GPIO.setup(l1, GPIO.OUT)
GPIO.setup(l2, GPIO.OUT)
GPIO.setup(medicine_pin,GPIO.OUT)
servo1=GPIO.PWM(medicine_pin,16)
GPIO.setup(leftSensor,GPIO.IN)
GPIO.setup(rightSensor,GPIO.IN)

#ultra sonic
GPIO.setup(TRIG_PIN , GPIO.OUT)
GPIO.setup(ECHO_PIN , GPIO.IN)	
GPIO.setup(BUZZER_PIN , GPIO.OUT)




                                    

def forward():
    print('forward')
    GPIO.output(r1,GPIO.HIGH)
    GPIO.output(r2,GPIO.LOW)
    GPIO.output(l1,GPIO.HIGH)
    GPIO.output(l2,GPIO.LOW)
    
    
    
def Left():
    GPIO.output(r1,GPIO.HIGH)
    GPIO.output(r2,GPIO.LOW)
    GPIO.output(l1,GPIO.LOW)
    GPIO.output(l2,GPIO.LOW)
    print('left')
    
    
def Right():
    GPIO.output(r1,GPIO.LOW)
    GPIO.output(r2,GPIO.LOW)
    GPIO.output(l1,GPIO.HIGH)
    GPIO.output(l2,GPIO.LOW)
    print('right')

def backward():
    GPIO.output(r1,GPIO.LOW)
    GPIO.output(r2,GPIO.HIGH)
    GPIO.output(l1,GPIO.LOW)
    GPIO.output(l2,GPIO.HIGH)
    print('back')
    
    
	
def stop():
    GPIO.output(r1,GPIO.LOW)
    GPIO.output(r2,GPIO.LOW)
    GPIO.output(l1,GPIO.LOW)
    GPIO.output(l2,GPIO.LOW)
    
    
def obstcals():
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
    


def move():
    while(True):
        if  GPIO.input(leftSensor)==0 and GPIO.input(rightSensor)==0:
            forward()
            print('forward')

        elif  GPIO.input(leftSensor)==0 and GPIO.input(rightSensor)==1:
             Left()
             print("left")
        elif  GPIO.input(leftSensor)==1 and GPIO.input(rightSensor)==0:
            Right()
            print("right")

        elif GPIO.input(leftSensor)==1 and GPIO.input(rightSensor)==1:
            stop()
            print("stop")
            #GPIO.cleanup()
            break
            #exit()

def return_to_base():
    right()
    time.sleep(5)
    stop_c=3
    while(stop_c!=0):
        move()
        forward()
        time.sleep(1)
        stop()
         





          
        
def reverse():
   backward()
   print('backward')
   time.sleep(3.35)
   stop()
   time.sleep(1)
   Right()
   time.sleep(4.6)
   stop()
   time.sleep(1)

    
d = input("Please enter Door Number : ")
f = input('Enter name of patient : ')



def QR_room():
    move()                      
    #stop_condition() 
    if searching(d) == 1:
        time.sleep(3)
        forward()
        #time.sleep(3)
        #stop()
        time.sleep(1)
        face_recognation()
    
    else:
        reverse()
        QR_room()
        


def face_recognation():
    move()
    forward()
    time.sleep(2)
    Right()
    time.sleep(2)
    stop()
    
    #code rana
    if face(f)==1:
        stop()
        #servo 
        servo1.start(7)
        time.sleep(1)
        servo1.ChangeDutyCycle(0)
        time.sleep(5)
        servo1.ChangeDutyCycle(1)
        time.sleep(1)
        servo1.stop()
    else:
        Left()
        time.sleep(3)
        stop()
        time.sleep(1)
        servo1.start(7)
        time.sleep(1)
        servo1.ChangeDutyCycle(0)
        time.sleep(7)
        servo1.ChangeDutyCycle(1)
        time.sleep(1)
        servo1.stop()
        
       

        

QR_room()