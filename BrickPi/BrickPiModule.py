from BrickPi import *
import time

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
        BrickPi.MotorEnable[PORT_D] = 1
        BrickPi.SensorType[PORT_1] = TYPE_SENSOR_TOUCH
        BrickPi.SensorType[PORT_2] = TYPE_SENSOR_LIGHT_ON
        BrickPi.SensorType[PORT_3] = TYPE_SENSOR_ULTRASONIC_CONT
        BrickPiSetupSensors()
        self._updated_ = False
    
    def avant(self, velocitat=50):
        velocitat = limita(velocitat)
        BrickPi.MotorSpeed[PORT_A] = velocitat
        BrickPi.MotorSpeed[PORT_D] = velocitat
        BrickPiUpdateValues()
        self._updated_ = True
        
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
            self._updated_ = True
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
