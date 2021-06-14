import cv2 #Libreria opencv
import numpy as np #Se importa numpy para trabajar con matrices de pixeles con opencv
from time import sleep #Tiempo de encendido de motores y led (el led se ocupa para encenderse cuando detecta un objeto y apagarse cuando sale de rango)
from gpiozero import AngularServo #Libreria del servo motor
from gpiozero import Robot #Libreria de motores de las llantas 
from gpiozero import LED #Libreria de led
#import os
#os.system("sudo modprobe bcm2835-v4l2")
servo = AngularServo(13, initial_angle= 90, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, frame_width=80/1000) #Se determinan el angulo inicial al cual se colocara el servo-motor y los angulos min y max, Asi como tambien los min y max de ancho de pulsos y se determina el tiempo de espacio entre pulso y pulso.  
robot = Robot(left=(22, 23), right=(17, 27), pwm=True) #Se declaran los 2 pines de la raspberry de cada lado, izquierdo y derecho. Asi como el pwm activado que sirve para regular la velocidad de las 
led = LED(24) #El led al pin 24
panAngle = 90 #Este algulo se coloca en 90 inicialmente debido a que se le tiene que sumar o restar ciertos grados respectivamente para moverlo de un lado a otro.
area = 0 #En este apartado se juega con el area del objeto y eso detrmina si el robot avanza se detiene y retrocede  
minimum_area = float(5000) #El area minima de 5000 pixeles del objeto a detectar, que funciona mediante un rectangulo con ciertos numeros de pixeles en su laterales y la multiplicacion de eso me dara el area. 
maximum_area = float(15000) #El area maxima de 15000 pixeles del objeto que se detecto dentro del rectangulo para despues ejecutar una accion.
cap = cv2.VideoCapture(0) #Videocaptura y se asigna el cero que corresponde a la camara de la raspberry
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480) #Se define el ancho y el alto de la ventana que se abre al encenderse la camara y se le asigna un valor de 480x320 pixeles.
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
cap.set(cv2.CAP_PROP_FPS, 25) #La cantidad FPS (fotogramas por segundo). El rango que se encontro fue de entre 25-30 ya que mas o menos el programa pierde rendimiento. 
redBajo1 = np.array([0, 150, 20], np.uint8) #Se determina el rango de colores con los que se detectara el objeto en este caso el rojo.
redAlto1 = np.array([10, 255, 255], np.uint8)
redBajo2 = np.array([175, 100, 20], np.uint8)
redAlto2 = np.array([179, 255, 255], np.uint8)
font = cv2.FONT_HERSHEY_SIMPLEX #Es para dar un mensaje cuando el objeto es detectado y cuando esta muy cerca dentro de la imagen de la camara.
while True:
    ret, frame = cap.read() #Verifica que el video se este grabando. 
    rows, cols, _ = frame.shape #.shape nos permite saber el numero de pixeles de ancho y alto y sabienso eso nos permite saber el centro de la pantalla que va relacionado con los servomotores para moverse de izq-der y tener una mejor vision del objeto detectado.
    center_image_x = int(cols/2)
    area = 100
    if(ret == True): #La condicional que verifica que este grabando, si es verdadero se ejecuta todo el proceso.
        frame = cv2.flip(frame, 1) #Se quita el modo espejo de la ventana de la camara  
        frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) #Se hace la conversion de RGB a HSV puesto que opencv trabaja con HSV.
        maskRed1 = cv2.inRange(frameHSV, redBajo1, redAlto1)
        maskRed2 = cv2.inRange(frameHSV, redBajo2, redAlto2)
        maskRed = cv2.add(maskRed1, maskRed2)
        contours, _ = cv2.findContours(maskRed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours: #Se detectara el objeto para enmarcarlo y a ese marco se le colocara un centroide 
            x, y, w, h = cv2.boundingRect(c) #Dibuja en tiempo real el marco que encerrara mi objeto.
            area = cv2.contourArea(c) #Se calcula el area del marco en tiempo real.
            Cx = int(x + (w/2)) #Se calcula el centroide a ese marco.
            Cy = int(y + (h/2)) 
            if(area > minimum_area and minimum_area < area < maximum_area): #Si esta dentro del area maxima y minima el objeto se encuentra detectado. Si mi objeto rebasa el area maxima se le coloca un aviso que el objeto esta muy cerca y si esta lejos saldra un mensaje que esta fuera de rango. 
                led.on() #Si esta detectado se prende el led. 
                    #sleep(0.0001)
                rectangle = cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 10)
                centroid = cv2.circle(frame,(Cx,Cy),7,(0,255,0),-1)
                coordinates = cv2.putText(frame,"{},{}".format(Cx,Cy), (Cx+10,300), font, 0.75,(0,255,0),1,cv2.LINE_AA)
                show_area = cv2.putText(frame,"{}".format(area), (350,30), font, 0.75,(0,255,0),1,cv2.LINE_AA)
                warning = cv2.putText(frame,"{}".format("OBJETO - DETECTADO"), (10,30), font, 0.75,(0,255,0),1,cv2.LINE_AA)
                if(Cx < center_image_x - 40):
                    print("Moviendo hacia la izquierda")
                    panAngle = panAngle - 1
                    if(panAngle < 90):
                        #robot.stop()
                        #sleep(0.05)
                        robot.value = (1, 1)
                        sleep(0.03)
                        robot.left(speed = 0.8)
                        sleep(0.06)
                        robot.value = (1, 1)
                        sleep(0.03)
                        #robot.stop()
                        #sleep(0.05)
                        robot.left(speed = 0.5)
                        sleep(0.05)
                        #robot.value = (1, 1)
                        #sleep(0.02)
                        #robot.left(speed = 0.2)
                        #sleep(0.01)
                    #else:
                        #robot.stop()
                        #sleep(0.05)
                        #robot.value = (-1, -1)
                        #sleep(0.05)
                elif(Cx > center_image_x + 40):
                    print("Movimiendo hacia la derecha")
                    panAngle = panAngle + 1
                    if(panAngle > 90):
                        #robot.stop()
                        #sleep(0.05)
                        robot.value = (-1, -1)
                        sleep(0.03)
                        robot.right(speed = 0.8)
                        sleep(0.06)
                        robot.value = (-1, -1)
                        sleep(0.03)
                        #robot.stop()
                        #sleep(0.05)
                        robot.right(speed = 0.5)
                        sleep(0.05)
                        #robot.value = (-1, -1)
                        #sleep(0.02)
                        #robot.right(speed = 0.2)
                        #sleep(0.02)
                    #else:
                        #robot.stop()
                        #sleep(0.05)
                        #robot.value = (-1, -1)
                        #sleep(0.05)
                else:
                    print("Moviendo al centro")
                    robot.right(speed = 0.9)
                    sleep(0.08)
            elif(area > minimum_area and area > maximum_area):
                led.off()
                #robot.stop()
                #sleep(0.05)
                #robot.left()
                #sleep(0.04)
                #robot.stop()
                #sleep(0.05)
                robot.left()
                sleep(0.06)
                show_area = cv2.putText(frame,"{}".format(area), (350,30), font, 0.75,(0,255,0),1,cv2.LINE_AA)
                warning = cv2.putText(frame,"{}".format("OBJETO - MUY CERCA"), (10,30), font, 0.75,(0,255,0),1,cv2.LINE_AA)
            else:
                led.off()
                robot.stop()
                #sleep(0.01)
        if(0 < panAngle < 180):
            servo.angle = panAngle
        cv2.imshow("RobotColor : Rojo", frame)
        if cv2.waitKey(1) & 0xFF == ord("s"):
            break
cap.release()
cv2.destroyAllWindows()
cv2.waitKey(10)
sleep(0.1)
cv2.waitKey(10)
cv2.waitKey(10)