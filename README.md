import re
import os
import cv2
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.adapters import common
from pycoral.adapters import classify
import pigpio
import time
import RPi.GPIO as GPIO
import numpy as np



modelPath = '/home/admin/Desktop/TF/model_edgetpu.tflite'


labelPath = '/home/admin/Desktop/TF/labels.txt'
GPIO.setmode(GPIO.BCM)
servo1 = 23
servo2 = 24
servo3 = 25
servo4 = 21
servo5 = 16
servo6 = 20
GPIO_TRIGGER = 6
GPIO_ECHO = 5
pwm = pigpio.pi()
pwm.set_mode(servo1, pigpio.OUTPUT)
pwm.set_mode(servo2, pigpio.OUTPUT)
pwm.set_mode(servo3, pigpio.OUTPUT)
pwm.set_mode(servo4, pigpio.OUTPUT)
pwm.set_mode(servo5, pigpio.OUTPUT)
pwm.set_mode(servo6, pigpio.OUTPUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

pwm.set_PWM_frequency(servo1, 50)
pwm.set_PWM_frequency(servo2, 50)
pwm.set_PWM_frequency(servo3, 50)
pwm.set_PWM_frequency(servo4, 50)
pwm.set_PWM_frequency(servo5, 50)
pwm.set_PWM_frequency(servo6, 50)
pwm.set_servo_pulsewidth(servo1, 500)
pwm.set_servo_pulsewidth(servo2, 500)
pwm.set_servo_pulsewidth(servo3, 500)
pwm.set_servo_pulsewidth(servo4, 500)
pwm.set_servo_pulsewidth(servo5, 500)
pwm.set_servo_pulsewidth(servo6, 500)

def distance():
   
    GPIO.output(GPIO_TRIGGER, True)

    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
 
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
   
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    TimeElapsed = StopTime - StartTime

    distance = (TimeElapsed * 34300) / 2
 
    return distance


def classifyImage(interpreter, image):
    size = common.input_size(interpreter)
    common.set_input(interpreter, cv2.resize(image, size, fx=0, fy=0,interpolation=cv2.INTER_CUBIC))
    interpreter.invoke()
    return classify.get_classes(interpreter)

def main():

    interpreter = make_interpreter(modelPath)
    interpreter.allocate_tensors()
    labels = read_label_file(labelPath)

    cap = cv2.VideoCapture(0)
    last_detection_cam = 0
    last_detection_metal = 0
    last_detection_kagit = 0
    last_detection_kompost = 0
    last_detection_izmarit = 0
    last_detection_plastik = 0 

    
    
    ilk_an = False
    dl = 0
    dolu =0
    while True:
        dist = distance()
        print("Measured Distance = %.1f cm" % dist)
        time.sleep(0.2)
        if dist < 15 and dolu == 0 and not ilk_an:
            ilk_an = True
            if ilk_an:
                dl = time.time()
        elif dist < 15 and time.time() - dl > 5:
            print('Stop')
            dolu = 1
            dl = 0
            ilk_an = False
        elif dist>15:
            dolu = 0
            dl = 0
            ilk_an = False
           
        ret, frame = cap.read()


        frame = cv2.flip(frame, 1)

       
        results = classifyImage(interpreter, frame)
        cv2.imshow('frame', frame)
        

        

        detected_label = labels[results[0].id]
        confidence_score = results[0].score
        print(f'Detected Label: {detected_label}, Score: {confidence_score}')
        
        if detected_label == 'glass' and confidence_score > 0.9:
            pwm.set_servo_pulsewidth(servo1, 1700)
            last_detection_cam = time.time()

        elif detected_label == 'metal' and confidence_score > 0.9:
            pwm.set_servo_pulsewidth(servo2, 1700)
            last_detection_metal = time.time()
        elif detected_label == 'kompost' and confidence_score > 0.9:
            pwm.set_servo_pulsewidth(servo3, 1700)
            last_detection_kompost = time.time()
        elif detected_label == 'kağıt' and confidence_score > 0.9:
            pwm.set_servo_pulsewidth(servo4, 1700)
            last_detection_kagit = time.time()
        elif detected_label == 'plastic' and confidence_score > 0.9:
            pwm.set_servo_pulsewidth(servo5, 1700)
            last_detection_plastik = time.time()
        elif detected_label == 'izmarit' and confidence_score > 0.9:
            pwm.set_servo_pulsewidth(servo6, 1700)
            last_detection_izmarit = time.time()
        
        if time.time() - last_detection_cam > 4:
            pwm.set_servo_pulsewidth(servo1, 500)
        if time.time() - last_detection_metal > 4:
            pwm.set_servo_pulsewidth(servo2, 500)
        if time.time() - last_detection_kompost > 4:
            pwm.set_servo_pulsewidth(servo3, 500)
        if time.time() - last_detection_kagit > 4:
            pwm.set_servo_pulsewidth(servo4, 500)
        if time.time() - last_detection_plastik > 4:
            pwm.set_servo_pulsewidth(servo5, 500)
        if time.time() - last_detection_izmarit > 4:
            pwm.set_servo_pulsewidth(servo6, 500)
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if _name_ == "_main_":
    main()
