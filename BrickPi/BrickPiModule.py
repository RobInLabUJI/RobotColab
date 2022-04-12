from BrickPi import *
import time
import picamera
import picamera.array
import cv2
import numpy as np

def limita(velocitat):
    velocitat = abs(velocitat)
    if velocitat<0:
        velocitat = 0
        if velocitat>100:
            velocitat = 100
    return velocitat
    
class BrickPiRobot:
    def __init__(self):
        BrickPiSetup()
        BrickPi.MotorEnable[PORT_A] = 1
        BrickPi.MotorEnable[PORT_C] = 1
        BrickPi.MotorEnable[PORT_D] = 1
        BrickPi.SensorType[PORT_1] = TYPE_SENSOR_TOUCH
        BrickPi.SensorType[PORT_2] = TYPE_SENSOR_LIGHT_ON
        BrickPi.SensorType[PORT_3] = TYPE_SENSOR_ULTRASONIC_CONT
        BrickPiSetupSensors()
        self._updated_ = False
    
    def velocitat_rodes(self, v_esquerra=0, v_dreta=0):
        if v_esquerra < -100:
            v_esquerra = -100
        elif v_esquerra > 100:
            v_esquerra = 100
        if v_dreta < -100:
            v_dreta = -100
        elif v_dreta > 100:
            v_dreta = 100
        BrickPi.MotorSpeed[PORT_A] = v_esquerra
        BrickPi.MotorSpeed[PORT_D] = v_dreta
        BrickPiUpdateValues()
        
    def avant(self, velocitat=50):
        velocitat = limita(velocitat)
        BrickPi.MotorSpeed[PORT_A] = velocitat
        BrickPi.MotorSpeed[PORT_D] = velocitat
        BrickPiUpdateValues()
        
    def arrere(self, velocitat=50):
        velocitat = limita(velocitat)
        BrickPi.MotorSpeed[PORT_A] = -velocitat
        BrickPi.MotorSpeed[PORT_D] = -velocitat
        BrickPiUpdateValues()
    
    def esquerra(self, velocitat=50):
        velocitat = limita(velocitat)
        BrickPi.MotorSpeed[PORT_A] = -velocitat
        BrickPi.MotorSpeed[PORT_D] = velocitat
        BrickPiUpdateValues()
    
    def dreta(self, velocitat=50):
        velocitat = limita(velocitat)
        BrickPi.MotorSpeed[PORT_A] = velocitat
        BrickPi.MotorSpeed[PORT_D] = -velocitat
        BrickPiUpdateValues()
    
    def pausa(self, t=1):
        for i in range(int(t*10)):
            BrickPiUpdateValues()
            time.sleep(0.1)
            
    def para(self):
        BrickPi.MotorSpeed[PORT_A] = 0
        BrickPi.MotorSpeed[PORT_D] = 0
        BrickPiUpdateValues()
        
    def tacte(self):
        BrickPiUpdateValues()
        return BrickPi.Sensor[PORT_1]
    
    def llum(self):
        BrickPiUpdateValues()
        return BrickPi.Sensor[PORT_2]
    
    def distancia(self):
        BrickPiUpdateValues()
        return BrickPi.Sensor[PORT_3]

    def camera_on(self):
        self.camera = picamera.PiCamera()
        self.camera.resolution = (640, 480)
        time.sleep(1)
        self.stream = picamera.array.PiRGBArray(self.camera)
        
    def imatge(self):
        self.stream.truncate(0)
        self.camera.capture(self.stream, format='rgb')
        image = self.stream.array
        return image
        
    def camera_off(self):
        self.camera.close()
        
    def detecta_foc(self, img, lower=200, upper=255):
        # region = img[250:,:,:]
        lower_tuple = (lower,lower,lower)
        upper_tuple = (upper,upper,upper)
        mask = cv2.inRange(img, lower_tuple, upper_tuple)
        _, cnt, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnt:
            cnt.sort(key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect( np.vstack(tuple(cnt)) )
            if w>1 and h>1:
                x += w/2
                y += h/2
                return x, y, w, h
            else:
                return None, None, None, None
        else:
            return None, None, None, None
        
    def ventilador_on(self, velocitat=255):
        BrickPi.MotorSpeed[PORT_C] = velocitat
        BrickPiUpdateValues()

    def ventilador_off(self):
        BrickPi.MotorSpeed[PORT_C] = 0
        BrickPiUpdateValues()
