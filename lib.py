import numpy as np
import cv2
import math
import RPi.GPIO as GPIO

def out(img):
    cv2.imshow("img",img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
class img_module:
    def out(self,img):
        cv2.imshow("img",img)
        cv2.waitKey()
        cv2.destroyAllWindows()
    def __init__(self):
        self.angle_tolerance = 0.2
        self.cap = cv2.VideoCapture(0)
    def flush(self):
        _,img = self.cap.read()
    def read(self):
        self.flush()
        _,img = self.cap.read()
        self.img = cv2.fastNlMeansDenoisingColored(img)
    def get_img_target(self):
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
    
# set pin numbers
L_pwm = 12
R_pwm = 13
frequency = 50 # in Hz  
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
    L.changeDutyCycle(speed)
    
def R_speed(speed): # speed in percentage (0~100)
    R.changeDutyCycle(speed)
    
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