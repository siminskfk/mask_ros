#!/usr/bin/env python3
# -------------------- Import -------------------- #

from tensorflow.keras.applications.mobilenet_v2 import preprocess_input
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
from imutils.video import VideoStream
import numpy as np
import imutils
import time
import cv2
import os
import rospy

from std_msgs.msg import Int32

rospy.init_node('test')
pub = rospy.Publisher('mask', Int32)


# load the face detector model
print("[INFO] loading face detector model...")
prototxtPath = "/root/catkin_ws/src/mask/opencv_face/deploy.prototxt"
weightsPath = os.path.sep.join(['/root/catkin_ws/src/mask/opencv_face/', "res10_300x300_ssd_iter_140000.caffemodel"])
face_detector = cv2.dnn.readNet(prototxtPath, weightsPath)

# etc
font = cv2.FONT_HERSHEY_SIMPLEX

# -------------------- Code for Mask Detecting -------------------- #

def detect_and_predict_mask(frame, face_detector):
    start = time.time()
    result = 0

    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), (104.0, 177.0, 123.0))
    face_detector.setInput(blob)
    detections = face_detector.forward()

    # initialize list of faces, locations, and predictions
    faces = []
    locs = []
    preds = []
    
    # loop for face
    for i in range(0, detections.shape[2]):
        # extract confidence 
        confidence = detections[0, 0, i, 2]

        # filter out weak detections
        if confidence > 0.5:
            # compute the (x, y)-coordinates of the bounding box 
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            # ensure the bounding boxes fall within the frame dimensions
            (startX, startY) = (max(0, startX), max(0, startY))
            (endX, endY) = (min(w - 1, endX), min(h - 1, endY))
            cv2.rectangle(frame, (startX, startY), (endX, endY), (0,0,255), 2)

            result += 1

    fps = 1 / (time.time() - start)
    fps = "FPS : %0.2f" %fps
    cv2. putText(frame, fps, (0, 20), font, 0.45, (0, 0, 0), 1)

    if result > 0 :
        pub.publish(result)
        time.sleep(3)
    
    return frame, result

# -------------------- Main -------------------- #

if __name__ == "__main__":

    print("[INFO] starting video stream...")
    cap = VideoStream(src=0).start()

    # loop over the frames from the video stream
    while True:
        frame = cap.read()
        frame = imutils.resize(frame, width=400)
        frame, result = detect_and_predict_mask(frame, face_detector)

        # display the output frame
        cv2.imshow("Frame", frame)
        
        # if the `q` key was pressed, break from the loop
        if cv2.waitKey(1) & 0xFF  == ord("q"):
            break
    
    cv2.destroyAllWindows()
    cap.stop()
