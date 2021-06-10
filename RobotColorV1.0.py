import cv2
import numpy as np
from time import sleep
from gpiozero import AngularServo
from gpiozero import Robot
from gpiozero import LED
#import os
#os.system("sudo modprobe bcm2835-v4l2")
servo = AngularServo(13, initial_angle= 90, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, frame_width=80/1000)
robot = Robot(left=(22, 23), right=(17, 27), pwm=True)
led = LED(24)
panAngle = 90
area = 0
minimum_area = float(5000)
maximum_area = float(15000)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
cap.set(cv2.CAP_PROP_FPS, 25)
redBajo1 = np.array([0, 150, 20], np.uint8)
redAlto1 = np.array([10, 255, 255], np.uint8)
redBajo2 = np.array([175, 100, 20], np.uint8)
redAlto2 = np.array([179, 255, 255], np.uint8)
font = cv2.FONT_HERSHEY_SIMPLEX
while True:
    ret, frame = cap.read()
    rows, cols, _ = frame.shape
    center_image_x = int(cols/2)
    area = 100
    if(ret == True):
        frame = cv2.flip(frame, 1)
        frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        maskRed1 = cv2.inRange(frameHSV, redBajo1, redAlto1)
        maskRed2 = cv2.inRange(frameHSV, redBajo2, redAlto2)
        maskRed = cv2.add(maskRed1, maskRed2)
        contours, _ = cv2.findContours(maskRed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            area = cv2.contourArea(c)
            Cx = int(x + (w/2)) 
            Cy = int(y + (h/2)) 
            if(area > minimum_area and minimum_area < area < maximum_area):
                led.on()
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