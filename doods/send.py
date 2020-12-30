#!/usr/bin/python3
import cv2 
import sys
import requests
import base64
import json
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import numpy as np
import time
colors = [(0, 255, 0) , (255, 0, 0) , (0, 0, 255) , (255, 255, 0) , (255, 0, 255) , (0, 255, 255) , (255, 0, 100) , (255, 100, 0) ,(100, 0, 255), (0, 0, 0) ]
num_colors = len(colors)
labels = []

CV_IMG = None
def send_img(image):
    url = 'http://192.168.1.136:8080/detect'
    retval, buffer = cv2.imencode('.jpg', image)
    jpg_as_text = base64.b64encode(buffer)
    myobj = {
    "detector_name": "default",
    "data": jpg_as_text,
        "detect": {
            "*": 60
        }
    }   
    x = requests.post(url, json = myobj)
    result = json.loads(x.text)
    return result

def publish_img(img, frame_data):
    msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    pub.publish(msg)
    rospy.loginfo("[+] publishing object detection ...")

def timer_callback(timer):
    if CV_IMG is None: return
    rospy.loginfo("[+] calling object detection ...")
    global labels
    start = time.time()
    
    h, w, _ = CV_IMG.shape 
    resp = send_img(CV_IMG)
    image = CV_IMG.copy()
    font = cv2.FONT_HERSHEY_SIMPLEX 
    
    thickness = 1
    fontScale = 0.85
    try:
        
        for obj in resp['detections']:
            print(obj)
            if not obj['label'] in labels:
                labels.append(obj['label'])
            label_index = labels.index(obj['label'])
            color = colors[label_index]
        #    start_point represents the top left corner of rectangle 
        #     end_point represents the bottom right corner of rectangle 
            start_point = (int(w*obj['left']), int(h*obj['top'])) 
            end_point = (int(w*obj['right']), int(h*obj['bottom'])) 
            print(start_point, end_point)
            image = cv2.rectangle(image, start_point, end_point, color, thickness) 
            image = cv2.putText(image, "{} = ({:.3f})".format(obj['label'], obj['confidence']) , start_point, font, fontScale, color, thickness, cv2.LINE_AA)
        elaspsed = time.time() - start
        # print(resp)
        print("time elapsed = {:3f}".format(elaspsed))
        # cv2.imshow('view', image)
        # cv2.waitKey(1)
        publish_img(image, json.dumps(obj) )
    except:
        rospy.logerr("cannot publish image")

        



def imgCallback(ros_data):
    global CV_IMG
    #### direct conversion to CV2 ####
    np_arr = np.fromstring(ros_data.data, np.uint8)
    CV_IMG = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    # print(CV_IMG.shape)

if __name__ == "__main__":
    # img = read_img(sys.argv[1])
    # print("sending pic ", img.shape)
    # print(send_img(img))
    rospy.init_node('object_detection2', disable_signals=True)
    rospy.loginfo('[+] object detection node is running')
    rospy.Subscriber('/uav_camera/compressed',CompressedImage, imgCallback, queue_size=1)
    pub = rospy.Publisher('/uav_camera/detected', Image, queue_size=1)
    timer = rospy.Timer(rospy.Duration(0.3), timer_callback)
    bridge = CvBridge()
    rospy.spin()
    