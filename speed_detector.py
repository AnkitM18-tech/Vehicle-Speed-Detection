#Importing Libraries

import cv2
import dlib
import time
import math

#Classifier File
carCascade = cv2.CascadeClassifier("vech.xml")

#Video file capture
video = cv2.VideoCapture("carsVideo.mp4")

# Constant Declaration
WIDTH =1280
HEIGHT = 720

#estimate speed function
def estimateSpeed(location1,location2):
    d_pixels = math.sqrt(math.pow(location2[0]-location1[0],2)+math.pow(location2[1]-location1[1],2))
    ppm = 8.8
    d_metres = d_pixels/ppm 
    fps = 18 
    speed = d_metres * fps * 3.6 
    return speed 

#detect multiple objects
def trackMultipleObjects():
    rectColor = (255,0,255)
    frameCounter = 0
    currentCarId = 0
    fps = 0 

    carTracker = {}
    carNumber = {}
    carLocation1 = {}
    carLocation2 = {}

    speed = [None]*1000 

    out = cv2.VideoWriter('outTraffic.avi',cv2.VideoWriter_fourcc('M','J','P','G'),10,(WIDTH,HEIGHT))

    while True:
        start_time = time.time()
        rc,frame = video.read()

        if type(frame) == type(None):
            break 

        frame = cv2.resize(frame,(WIDTH,HEIGHT))
        resultImage = frame.copy()
        frameCounter += 1
        carIDToDelete = []

        for carID in carTracker.keys():
            trackingQuality = carTracker[carID].update(frame)

            if trackingQuality < 7:
                carIDToDelete.append(carID)

        for carID in carIDToDelete:
            print("Removing "+str(carID)+" from list of trackers.")
            print("Removing "+str(carID)+" previous location.")
            print("Removing "+str(carID)+" current location.")

            carTracker.pop(carID,None)
            carLocation1.pop(carID,None)
            carLocation2.pop(carID,None)

        if  not (frameCounter % 10):
            grayScaleImage = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            cars = carCascade.detectMultiScale(grayScaleImage,1.1,13,18,(24,24))

            for (_x,_y,_w,_h) in cars :
                x = int(_x)
                y = int(_y)
                w = int(_w)
                h = int(_h)

                x_bar = x + 0.5*w
                y_bar = y + 0.5*h 

                matchCarID = None 

                for carID in carTracker.keys():
                    trackerPosition = carTracker[carID].get_position()

                    t_x = int(trackerPosition.left())
                    t_y = int(trackerPosition.top())
                    t_w = int(trackerPosition.width())
                    t_h = int(trackerPosition.height())

                    t_xbar = t_x + 0.5*t_w 
                    t_ybar = t_y + 0.5*t_h 

                    if((t_x <= x_bar <= (t_x + t_w)) and (t_y <= y_bar <= (t_y + t_h)) and (x <= t_xbar <= (x+w)) and (y <= t_ybar <= (y+ h))):
                        matchCarID = carID 

                if matchCarID is None:
                    print("Creating new tracker " + str(currentCarId))

                    tracker = dlib.correlation_tracker()
                    tracker.start_track(frame,dlib.rectangle(x,y,x+w,y+h))

                    carTracker[currentCarId] = tracker 
                    carLocation1[currentCarId] = x,y,w,h 
                    currentCarId  += 1
        
        for carID in carTracker.keys():
            trackedPosition = carTracker[carID].get_position()
            t_x = int(trackedPosition.left())
            t_y = int(trackedPosition.top())
            t_w = int(trackedPosition.width())
            t_h = int(trackedPosition.height())

            cv2.rectangle(resultImage,(t_x,t_y),(t_x + t_w,t_y + t_h),rectColor,4)

            carLocation2[carID] = [t_x,t_y,t_w,t_h]
        end_time = time.time()

        if not(end_time == start_time):
            fps = 1.0/(end_time - start_time)

        for i in carLocation1.keys():
            if frameCounter % 1 == 0:
                [x1,y1,w1,h1] = carLocation1[i]
                [x2,y2,w2,h2] = carLocation2[i]

                carLocation1[i] = [x2,y2,w2,h2]

                if [x1,y1,w1,h1] != [x2,y2,w2,h2]:
                    if (speed[i] == None or speed[i] == 0) and y1 >=275 and y1 <= 285:
                        speed[i] == estimateSpeed([x1,y1,w1,h1],[x2,y2,w2,h2])

                    if speed[i] != None and y >= 100:
                        cv2.putText(resultImage, str(int(speed[i]))+" km/h", int(x1 + w1/2), int(y1 - 5),cv2.FONT_HERSHEY_SIMPLEX,0.75,(0,0,100),2)

        cv2.imshow("Result",resultImage)
        out.write(resultImage)

        if cv2.waitKey(1) == 27:
            break
    
    cv2.destroyAllWindows()
    out.release()

if __name__ == '__main__':
    trackMultipleObjects()