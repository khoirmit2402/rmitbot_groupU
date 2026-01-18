# ED3 - Web Server and API interface
# Written by Do Tri Toan - B.Eng ECS
#
# Description: This program received information from the serial monitor over HTTP packets,
# then requests information from Mission Control (AppScript API) when apropriate
#
# Reference: https://www.geeksforgeeks.org/python/get-post-requests-using-python/
# This code was modified from the reference because the author does not know how to use Python :(

import requests
import json
import os
import subprocess

from bottle import post, run, request, static_file

# import classification script
import classify


### Getting data from AppScript API
def requestCoords(food):
    # api-endpoint
    URL = "https://script.google.com/macros/s/AKfycbzuOXHfDYYqWx8Tbq39Wf74i9e2xXl0em0dWbGHEDQRILctgLEtJSPasLod6ra_-DWFMg/exec"

    # messages to be sent
    origin = "pc"  # Do not change!
    message = str(food)  # to be changed to the output of CV

    # defining a params dict for the parameters to be sent to the API
    PARAMS = {'origin': origin, 'message': message}

    # sending get request and saving the response as response object
    r = requests.get(url=URL, params=PARAMS)

    # return data in text format
    return r.text


def sendRobothome():
    # api-endpoint
    URL = "https://script.google.com/macros/s/AKfycbzuOXHfDYYqWx8Tbq39Wf74i9e2xXl0em0dWbGHEDQRILctgLEtJSPasLod6ra_-DWFMg/exec"
    origin = "rpi"  # Do not change!
    message = "sendmehome!"  # to be changed to the output of CV

    # defining a params dict for the parameters to be sent to the API
    PARAMS = {'origin': origin, 'message': message}

    # sending get request and saving the response as response object
    r = requests.get(url=URL, params=PARAMS)

    # return data in text format
    return r.text

def navToCoords(coordsStr):
    ## PARSING INFO 
    coordsArr = coordsStr.split()
    # For troubleshooting, print coords
    
    ## RUNNING COMMAND IN TERMINAL
    process = subprocess.call(["ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ' { pose: { header: { frame_id: 'map' }, pose: { position: { x: ", str(coordsArr[0]) ,", y:  ", str(coordsArr[1]) , ", z: ", str(coordsArr[3]), "}, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } } }'"], shell = True)
    process.wait()

## WEB SERVER to send coords LOCALLY
@post('/coords')
def my_coords():
    req_obj = json.loads(request.body.read())
    return requestCoords(" ")

@post('/rpihome')
def myRPI():
    req_obj = json.loads(request.body.read())
    return sendRobothome()

# Tunnel for images
@post('/upload')
def myUpload():
    if os.path.exists("./abc.jpg"):
        os.remove("./abc.jpg")
    upload = request.files.get('file')
    save_path = "./"

    try:
        # Default Coords
        prediction = "0 0 0"
        filepath = "./abc.jpg"
        upload.save(filepath)

        prediction = classify.predict_food(filepath)
        print(classify.predict_food(filepath))
        coords = requestCoords(prediction)

        return(coords)

        #navToCoords(coords)
        #return("Command sent!")

    except IOError:
        return "Nice try!"

run(host='172.20.10.4', port=8000, debug=True)
