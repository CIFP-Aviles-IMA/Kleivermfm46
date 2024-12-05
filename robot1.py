"""Este script procesa el funcionamiento de control de un brazo robótico articulado. El brazo robotico compuesto por servomotores,
controlador PWM PCA9685 conectado a una placa Jetson. 
El brazo tiene varios servomotores para mover las articulaciones  y sensores de posición (potenciómetros) que permiten ajustar los servos.
Se pude controlar una de las partes del brazo articulado, este parte es la garra la cual se puede abrir o cerrar mediante un boton.

Biblioteca o Libreria(Importaciones necesarias para el funcionamiento del brazo robotico):
-Broad: se usa para acceder a pines de entrada/salida de una placa, en este caso la Jetson.
-Busio: proporciona soporte para la comunicación con dispositivos a través de buses como I2C.
-Jetson.GPIO: Se utiliza para controlar los pines GPIO en la placa Jetson.
-adafruit_pca9685: Permite la comunicación con el controlador PWM PCA9685.
-adafruit_servokit: Facilita la gestión de servomotores mediante la librería Adafruit.
-time: Se usa para añadir retrasos entre las acciones y configurar el sistema.

Procedimiento del script:
-Un botón conectado al pin GPIO 15 controla la garra del brazo. Si el botón no está presionado, la garra se cierra; cuando se presiona, la garra se abre.
-El script configura los servomotores utilizando el controlador PWM PCA9685, que envía las señales PWM necesarias para controlar la posición de cada motor.
-Los potenciómetros se leen a través de los pines GPIO, y sus valores se mapean a un rango que ajusta el ángulo de los servos en las articulaciones. 
Estos valores se convierten en un ancho de pulso PWM que controla el movimiento de los servos

Funcionamiento:
-MIN_PULSE_WIDTH: establece el pulso minimo para los servomotores (650 microsegundos)
-MAX_PULSE_WIDTH: establece el pulso máximo para el movimiento de los servomotores (2350 microsegundos).
-FREQUENCY: momnetos por segundo (50Hz) de actualización de la señal PWM 
-kit = ServoKit: reduce el nombre de la libreria y su funcion es la gestion de los servomotores
-moveMotor: segun el potenciometro ajusta el motor del servomotor
"""

#import Wire 
#import Adafruit_PWMServoDriver
import board
import busio
import Jetson.GPIO as GPIO
import adafruit_pca9685
i2c = busio.I2C(board.SCL, board.SDA)
from adafruit_servokit import ServoKit
import time 

#Declaro las variables globales
MIN_PULSE_WIDTH=    650
MAX_PULSE_WIDTH=    2350
FREQUENCY      =    50


#Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
kit = ServoKit(channels=16)


#Configuro el SetUP
time.sleep(5)                           #<-- So I have time to get controller to starting position
pwm.frequency = FREQUENCY
GPIO.setmode(GPIO.BOARD)
hand = adafruit_motor.servo.Servo(0)   #Cualquiera de las dos opciones
wrist = adafruit_motor.servo.Servo(1)
potWrist = adafruit_motor.servo.Servo(2) #//Assign Potentiometers to pins on Arduino Uno
elbow = adafruit_motor.servo.Servo(3) 
potElbow = adafruit_motor.servo.Servo(4)
shoulder = adafruit_motor.servo.Servo(5)
potShoulder = adafruit_motor.servo.Servo(6)
base = adafruit_motor.servo.Servo(7)                    
potBase = adafruit_motor.servo.Servo(8)


pwm.setPWMFreq(FREQUENCY)
pwm.setPWM(32, 0, 90)                  #Set Gripper to 90 degrees (Close Gripper) X en Jetson
pwm.begin()                            
GPIO.setup(11, GPIO.IN)          


#Asignamos pines
int potWrist    = A3
int potElbow    = A2                    #Assign Potentiometers to pins on Arduino Uno
int potShoulder = A1
int potBase     = A0

int hand      = 11
int wrist     = 12
int elbow     = 13                      #Assign Motors to pins on Servo Driver Board
int shoulder  = 14
int base      = 15


def moveMotor(controlIn, motorOut):
    """
    Descripción de la función def MoveMotor(controlIN, motorOUT):
    -controlIn (int): El pin GPIO entrada
    -motorOut (int): El pin GPIO salida. Este pin se utiliza para enviar la señal PWM al motor que se va a controlar.
    """

    pulse_wide, pulse_width, potVal = -7
  
    #potVal = analogRead(controlIn); (Lenguaje C)                                                  #Read value of Potentiometer
    potVal = GPIO.input(controlIn)
    pulse_wide = map(potVal, 800, 240, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)
    pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);                #Map Potentiometer position to Motor
  
    #pwm.setPWM(motorOut, 0, pulse_width); (Lenguaje C) 
    pwm = GPIO.PWM(motorOut, pulse widht)
 
while (True):
    moveMotor(potWrist, wrist)
    moveMotor(potElbow, elbow)            #Assign Motors to corresponding Potentiometers
    moveMotor(potShoulder, shoulder)
    moveMotor(potBase, base)
    int pushButton = GPIO.input(11)
    if(pushButton == GPIO.LOW):
    pwm.setPWM(hand, 0, 180)                             #Keep Gripper closed when button is not pressed
    print("Grab")
    else:
    pwm.setPWM(hand, 0, 90);                              #Open Gripper when button is pressed
    print("Release")
GPIO.cleanup()