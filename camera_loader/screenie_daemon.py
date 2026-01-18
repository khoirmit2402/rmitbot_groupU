# ED3 - Serial monitor and image capturer (@ Raspberry Pi)
# Written by Do Tri Toan - B.Eng ECS
#
# Description: This program monitors serial messages sent by the scale, taking a picture everytime something appears on the scale, and broadcasting its status when scale is zeroed.
#
# Reference: https://www.geeksforgeeks.org/python/how-to-capture-a-image-from-webcam-in-python/
#

import cv2
import requests
import json
import time

import serial_scraper
# serial_scraper has function readSerial(comport), which returns the message

def takePhoto():
    # Initialize webcam (0 = default camera)
    cam = cv2.VideoCapture(0)

    # Capture one frame
    ret, frame = cam.read()

    if ret:
        # cv2.imshow("Captured", frame)         
        cv2.imwrite("example.jpg", frame)  
        # cv2.waitKey(0)                      
        # cv2.destroyWindow("Captured")       
    else:
        print("Failed to capture image.")

    cam.release() 

# Sending image to webserver through
def sendImage():
    # Send image over HTTP
    url = "http://localhost:8000/upload"

    files={
            'file': (open('./example.jpg','rb')),
             'Content-Type': 'image/jpeg'
    }

    r = requests.post(url, files=files)
    return r.text

def sendData():
    # Send data over HTTP
    url = "http://localhost:8000/rpihome"

    r = requests.post(url, '{}')

    return r.text
    
if __name__ == "__main__":
    stat = 0
    while True: # Keep program running!
        data = serial_scraper.readSerial('/dev/ttyACM0')
        measure = float(data)
        print(measure)
        if (measure > 40):
            if (stat == 0):
               # Set stat to TRUE then take pictures
              stat = stat + 1
                takePhoto()
                coords = sendImage()
                navigateTo = coords
                print(navigateTo)
            else:
                print('Carrying out order!')

        if (measure > 0):
            if (measure < 1):
                if (stat == 1):
                    # set stat to FALSE, then inform Mission Control (API)
                    stat = stat - 1
                    navigateTo = sendData()
                    print('Going home!')
                    print(navigateTo)
        else:
            print('Idling')





