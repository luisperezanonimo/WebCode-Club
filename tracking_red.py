import cv2
import numpy as np
minimum_area = float(3000)
maximum_area = float(30000)
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
    if(ret == True):
        frame = cv2.flip(frame, 1)
        frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        maskRed1 = cv2.inRange(frameHSV, redBajo1, redAlto1)
        maskRed2 = cv2.inRange(frameHSV, redBajo2, redAlto2)
        maskRed = cv2.add(maskRed1, maskRed2)
        contours, _ = cv2.findContours(maskRed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            object_area = cv2.contourArea(c)
#           object_area = w*h
            Cx = int(x + (w/2))
            Cy = int(y + (h/2))
            if(minimum_area < object_area < maximum_area):
                rectangle = cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 10)
                centroid = cv2.circle(frame,(Cx,Cy),7,(0,255,0),-1)
                coordinates = cv2.putText(frame,"{},{}".format(Cx,Cy), (Cx+10,300), font, 0.75,(0,255,0),1,cv2.LINE_AA)
                show_area = cv2.putText(frame,"{}".format(object_area), (350,30), font, 0.75,(0,255,0),1,cv2.LINE_AA)
                warning = cv2.putText(frame,"{}".format("OBJECT - DETECTED"), (10,30), font, 0.75,(0,255,0),1,cv2.LINE_AA)   
            elif(object_area > maximum_area):
                show_area = cv2.putText(frame,"{}".format(object_area), (350,30), font, 0.75,(0,255,0),1,cv2.LINE_AA)
                warning = cv2.putText(frame,"{}".format("OBJECT - TOO CLOSE"), (10,30), font, 0.75,(0,255,0),1,cv2.LINE_AA)
            """This next commented block is the problem i guess, i dont know where is the mistake"""
            #elif(object_area < minimum_area):
                #show_area = cv2.putText(frame,"{}".format(object_area), (350,30), font, 0.75,(0,255,0),1,cv2.LINE_AA)
                #warning = cv2.putText(frame,"{}".format("OBJECT - TOO FAR"), (10,30), font, 0.75,(0,255,0),1,cv2.LINE_AA)
        cv2.imshow("Tracking RED Objects", frame)
        if cv2.waitKey(1) & 0xFF == ord("s"):
            break
cap.release()
cv2.destroyAllWindows()
