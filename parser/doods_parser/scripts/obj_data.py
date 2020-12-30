#!/usr/bin/python3
import cv2 
import sys
import requests
import base64
import json
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import threading
import time


labels = ['person', 'chair']
CV_IMG = None

def send_img(image):
    """
    :param image: cv image
    :return: make an http request to the object detection server
    """
    url = 'http://192.168.1.136:8080/detect'
    retval, buffer = cv2.imencode('.jpg', image)
    jpg_as_text = base64.b64encode(buffer).decode('utf-8')
    myobj = {
    "detector_name": "default",
    "data": jpg_as_text,
        "detect": {
            "*": 60
        }
    }   
    x = requests.post(url, json = myobj)
    result = json.loads(x.text)
    return result["detections"]

def publish_data(json_data):
    string_data = json.dumps(json_data)
    msg = String()
    msg.data = string_data
    pub.publish(msg)
    rospy.loginfo("[+] publishing ... %s"% string_data)


def get_bounding_box(image, y_pred):
    """
    :param image: HD image from usb camera
    :param y_pred: prediction from object detector
    :return: overlay image bounding box and label
    """
    assert len(image.shape) == 3
    h, w, _ = image.shape
    start_point = (int(w*y_pred['left']), int(h*y_pred['top']))
    end_point = (int(w*y_pred['right']), int(h*y_pred['bottom']))
    data = { 'label':y_pred['label'], 'conf' :y_pred['confidence'], 'box':[ start_point[0], start_point[1], end_point[0], end_point[1] ] }
    return data 

def timer_callback(timer):
    if CV_IMG is None: return
    # rospy.loginfo("[+] calling object detection ...")
    global labels
    lock.acquire()
    # header = copy(CV_HEADER)
    try:
        resp = send_img(CV_IMG)
        # publish_data(resp, header)
        data = [get_bounding_box(CV_IMG, r) for r in resp]
        publish_data(data)
    except:
        rospy.logerr("[!] %s "%sys.exc_info()[0])
    finally:
        lock.release()

        



def imgCallback(ros_data):
    global CV_IMG
    #### direct conversion to CV2 ####
    # np_arr = np.fromstring(ros_data.data, np.uint8)
    # CV_IMG = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    lock.acquire()
    CV_IMG = bridge.compressed_imgmsg_to_cv2(ros_data)
    lock.release()
    # print(CV_IMG.shape)

if __name__ == "__main__":
    rospy.init_node('object_detection2', disable_signals=True)

    bridge = CvBridge()
    lock = threading.Lock()
    rospy.Subscriber('/uav_camera/compressed',CompressedImage, imgCallback, queue_size=1)
    pub = rospy.Publisher('/uav_camera/objects', String, queue_size=1)
    pub_rate = rospy.get_param('~pub_rate', 0.3)
    rospy.loginfo('[+] object detection node is running at %d Hz'%int(1/pub_rate))
    timer = rospy.Timer(rospy.Duration(pub_rate), timer_callback)
    rospy.spin()
    