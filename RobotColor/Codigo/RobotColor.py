import cv2
import numpy as np
from time import sleep
from gpiozero import AngularServo
from gpiozero import Robot
#import os
#IN1 -> GREEN -> GPIO17
#IN2 -> YELLOW -> GPIO27
#IN3 -> ORANGE -> GPIO22
#IN4 -> RED -> GPIO23
#os.environ["DISPLAY"] = "localhost:10.0"
servo = AngularServo(13, initial_angle= 90, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, frame_width=80/1000)
robot = Robot(left=(22, 23), right=(17, 27), pwm=True)
global panAngle
global area
global Cx
panAngle = 90
area = 0
minimum_area = 3500 
maximum_area = 20000
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
cap.set(cv2.CAP_PROP_FPS, 30)
redBajo1 = np.array([0, 100, 20], np.uint8)
redAlto1 = np.array([10, 255, 255], np.uint8)
redBajo2 = np.array([175, 100, 20], np.uint8)
redAlto2 = np.array([179, 255, 255], np.uint8)
font = cv2.FONT_HERSHEY_SIMPLEX
while True:
    ret, frame = cap.read()
    rows, cols, _ = frame.shape
    center_image_x = int(cols/2)
    if(ret == True):
        frame = cv2.flip(frame, 1)
        frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        maskRed1 = cv2.inRange(frameHSV, redBajo1, redAlto1)
        maskRed2 = cv2.inRange(frameHSV, redBajo2, redAlto2)
        maskRed = cv2.add(maskRed1, maskRed2)
        contornos, _ = cv2.findContours(maskRed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contornos:
            area = cv2.contourArea(c)
            if(area > minimum_area):
                if(minimum_area < area < maximum_area):
                    M = cv2.moments(c)
                    if(M["m00"]==0):
                        M["m00"]=1
                    Cx = int(M["m10"]/M["m00"])
                    Cy = int(M["m01"]/M["m00"])
                    x, y, w, h = cv2.boundingRect(c)
                    area_encontrada = w*h
                    centroide = cv2.circle(frame,(Cx,Cy),7,(0,255,0),-1)
                    coordenadas = cv2.putText(frame,"{},{}".format(Cx,Cy), (Cx+10,300), font, 0.75,(0,255,0),1,cv2.LINE_AA)
                    warning_0 = cv2.putText(frame,"{}".format("OBJETO - DETECTADO"), (10,30), font, 0.75,(0,255,0),1,cv2.LINE_AA)
                    show_area_1 = cv2.putText(frame,"{}".format(area), (350,30), font, 0.75,(0,255,0),1,cv2.LINE_AA)
                    rectangulo = cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 10)
                    if(Cx < center_image_x - 40):
                        panAngle = panAngle - 1
                        print("Moviendo hacia la izquierda")
                    elif(Cx > center_image_x + 40):
                        panAngle = panAngle + 1
                        print("Movimiendo hacia la derecha")
                    else:
                        print("Moviendo al centro")
                        robot.value = (0.5, -0.5)
#                         sleep(10/1000000)
                    if(0 <= panAngle <= 180):
                        servo.angle = panAngle
#                         sleep(1/1000000)
                elif(area > maximum_area):
                    warning_1 = cv2.putText(frame,"{}".format("OBJETO - MUY CERCA"), (10,30), font, 0.75,(0,255,0),1,cv2.LINE_AA)
                    show_area_2 = cv2.putText(frame,"{}".format(area), (350,30), font, 0.75,(0,255,0),1,cv2.LINE_AA)
                    #robot.value = (0.5, -0.5)
#                     sleep(100/1000)
                    robot.stop()
                elif(minimum_area > area):
                    robot.stop()
                    sleep(1/1000000)
        cv2.imshow("Detectando color ROJO", frame)
        if cv2.waitKey(1) & 0xFF == ord("s"):
            break
cap.release()
cv2.destroyAllWindows()