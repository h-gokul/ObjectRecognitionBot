'''

This is the header file for the main driver of the bot.

This contains the functions and modules used to control the bot on the lower level

''' 
#header files needed

import numpy as np
import math
import cv2
import time
# Rpi headers
import RPi.GPIO as GPIO
from picamera.array import PiRGBArray
from picamera import PiCamera

def out(img): #to display a image
    cv2.imshow("img",img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
# image processing module to be used

class img_module:
    def read_img_pi(self): # to read image from rpi
        self.cam.capture(self.raw,format="bgr")
        image = self.raw.array
        return image;
    def __init__(self): #init parameters
        self.angle_tolerance = 0.2
        self.cap = cv2.VideoCapture(0)
        self.cam = PiCamera()
        self.rawCam = rawRGBArray(self.cam)
        time.sleep(0.1) # to allow camera init 
        
    def flush(self): #flush in laptop // where VideoCapture is enabled
        _,img = self.cap.read()
        
    def read(self): # read img in laptop // where VideoCapture is enabled
        self.flush()
        _,img = self.cap.read()
        self.img = cv2.fastNlMeansDenoisingColored(img)
        
    def read_pi(self): #rpi support for videoCapture not working
        read_img_pi()
        self.img = read_img_pi()
        self.img = cv2.fastNlMeansDenoisingColoured(self.img)
        
    def get_img_target(self): #to detect a ball and go near it
        gray = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
        circles = np.uint16(np.around(circles))
        try:
            i = circles[0,0]
        except:
            return "object_not_found"
        centroid = (i[0],i[1])
        hor = i[0] #longer side
        ver = i[1] # shorter side
        copy = self.img.copy()
        cv2.circle(copy,(i[0],i[1]),i[2],(0,255,0),2)
        dist = (((320 - hor)*(320 - hor)) + ((480-ver)*(480-ver)))**0.5
        angle = math.atan((320-hor)/(480-ver)) # -ve if object is left ||  +ve if right
        cv2.circle(copy,(320,480),2,(0,255,0),-1)
        out(copy)
        return (centroid,(dist,i[2]),angle,copy)
    
'''
Camera for Pi:
The PiCam module cannot be accessed using the VideoCapture class. Getting this to work is very tricky.
The code contains an alternative method to read the image from the PiCam 

Pin Connections:

pwm 0 (bcm 12) and bcm 5 control left motor
pwm 1 (bcm 13) and bcm 6 control right motor
common ground at pin 34
PiCam connected and enabled

Pin numbers provided below is the bcm pin number

'''    
    
# set pin numbers for the GPIO pins // robot actuation
L_pwm = 12
R_pwm = 13
frequency = 100 # in Hz  
L_dir = 5  # direction
R_dir = 6  #direction
fast = 40 #fast speed duty cycle
slow = 20 #slow speed duty cycle

'''
Logic for actuation:

Rpi has 2 pwn outputs, but 4 pins for it.
we tried to use a separate relay circuit but the bot can only move if the voltage output from the controlling module is 5V.
Rpi however only supplies 3.3V. 
So we use a L298 module as it is more sensitive the original controls have been deprecated and provided in the comments below

forward ==> dir=0 and duty cycle given
reverse ==> dir=1 and duty cycle = 100-given ( (1,1) setting is halt)

'''

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(L_dir,GPIO.OUT)
    GPIO.setup(R_dir, GPIO.OUT)
    GPIO.setup(L_pwm, GPIO.OUT)
    GPIO.setup(R_pwm, GPIO.OUT)
    L = GPIO.PWM(L_pwm,frequency)
    R = GPIO.PWM(R_pwm,frequency)
    L.start(0)
    R.start(0)
    return (L,R)

(L,R) = setup()

# Lower level controls for actuation

def slow_left_forward():
    L .ChangeDutyCycle(slow)
    GPIO.output(L_dir, GPIO.LOW)

def slow_left_reverse():
    L .ChangeDutyCycle(100 - slow)
    GPIO.output(L_dir, GPIO.HIGH)

def fast_left_forward():
    L .ChangeDutyCycle(fast)
    GPIO.output(L_dir, GPIO.LOW)

def fast_left_reverse():
    L .ChangeDutyCycle(100 - fast)
    GPIO.output(L_dir, GPIO.HIGH)
    
def slow_right_forward():
    R .ChangeDutyCycle(slow)
    GPIO.output(R_dir, GPIO.LOW)

def slow_right_reverse():
    R .ChangeDutyCycle(100 - slow)
    GPIO.output(R_dir, GPIO.HIGH)

def fast_right_forward():
    R.ChangeDutyCycle(fast)
    GPIO.output(R_dir, GPIO.LOW)

def fast_right_reverse():
    R.ChangeDutyCycle(100 - fast)
    GPIO.output(R_dir, GPIO.HIGH)


# Higher level calls to be used in the driver code

def forward_slow():
    slow_left_forward()
    slow_right_forward()
    
def forward_fast():
    fast_left_forward()
    fast_right_forward()
    
def turn_left():
    slow_left_reverse()
    slow_right_forward()
    
def turn_right():
    slow_right_reverse()
    slow_left_forward()
    
def glide_left():
    slow_left_forward()
    fast_right_forward()
    
def glide_right():
    fast_left_forward()
    slow_right_forward()
    

'''
this works with relay module perfectly.
RPi output voltage doesn't work with implemented circuitry. So moving on to using L298 module and changes in driver code
### Ignore 
L_relay_0 = 23 
L_relay_1 = 24
R_relay_0 = 27 
R_relay_1 = 22

def setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(L_relay_0, GPIO.OUT)
    GPIO.setup(R_relay_0, GPIO.OUT)
    GPIO.setup(L_relay_1, GPIO.OUT)
    GPIO.setup(R_relay_1, GPIO.OUT)
    GPIO.setup(L_pwm, GPIO.OUT)
    GPIO.setup(R_pwm, GPIO.OUT)
    L = GPIO.PWM(L_pwm,frequency)
    R = GPIO.PWM(R_pwm,frequency)
    L.start(0)
    R.start(0)
    return (L,R)
    
(L,R) = setup();


def R_stop():
    GPIO.output(R_relay_1, GPIO.LOW)
    GPIO.output(R_relay_0, GPIO.LOW)

def L_stop():
    GPIO.output(L_relay_0, GPIO.LOW)
    GPIO.output(L_relay_1, GPIO.LOW)
    
def L_forward():
    GPIO.output(L_relay_0, GPIO.LOW)
    GPIO.output(L_relay_1, GPIO.HIGH)
    
def R_forward():
    GPIO.output(R_relay_0, GPIO.LOW)
    GPIO.output(R_relay_1, GPIO.HIGH)
    
def L_reverse():
    GPIO.output(L_relay_0, GPIO.HIGH)
    GPIO.output(L_relay_1, GPIO.LOW)
    
def R_reverse():
    GPIO.output(R_relay_0, GPIO.HIGH)
    GPIO.output(R_relay_1, GPIO.LOW)
    
def L_speed(speed): # speed in percentage (0~100)
    L.ChangeDutyCycle(speed)
    
def R_speed(speed): # speed in percentage (0~100)
    R.ChangeDutyCycle(speed)
    
def move_left():
    L_reverse()
    R_forward()
    
def move_right():
    R_reverse()
    L_forward()
    
def move_front():
    R_forward()
    L_forward()
    
def stop():
    L_stop()
    R_stop()
'''

