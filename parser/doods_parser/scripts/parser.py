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
import threading
import time
colors = [(0, 255, 0) , (255, 0, 0) , (0, 0, 255) , (255, 255, 0) , (255, 0, 255) , (0, 255, 255) , (255, 0, 100) , (255, 100, 0) ,(100, 0, 255), (0, 0, 0) ]
num_colors = len(colors)
labels = []
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
    return result

def publish_img(img, pred):
    """
    :param img: overlaid image
    :param frame_data: raw object detection in json string
    :return: publish image which is then converted into a compressed image using image_transporter
    """
    rospy.loginfo("[+] publishing object detection ...")
    msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    msg.header.frame_id = json.dumps(pred)
    pub.publish(msg)



def draw_bounding_box(image, y_pred):
    """
    :param image: HD image from usb camera
    :param y_pred: prediction from object detector
    :return: overlay image bounding box and label
    """
    assert len(image.shape) == 3
    h, w, _ = image.shape
    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = 1
    fontScale = 0.85
    label_index = labels.index(y_pred['label'])
    color = colors[label_index]
    start_point = (int(w*y_pred['left']), int(h*y_pred['top']))
    end_point = (int(w*y_pred['right']), int(h*y_pred['bottom']))
    image = cv2.rectangle(image, start_point, end_point, color, thickness)
    image = cv2.putText(image, "{} = ({:.3f})".format(y_pred['label'], y_pred['confidence']) , start_point, font, fontScale, color, thickness, cv2.LINE_AA)
    return image

def timer_callback(timer):
    if CV_IMG is None: return
    rospy.loginfo("[+] calling object detection ...")
    global labels
    lock.acquire()
    try:
        resp = send_img(CV_IMG)
        print(resp)
        image = CV_IMG.copy()
        for obj in resp['detections']:
            if not obj['label'] in labels:
                labels.append(obj['label'])
            image = draw_bounding_box(image, obj)
        publish_img(image, resp['detections'])
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
    pub = rospy.Publisher('/uav_camera/detected', Image, queue_size=1)
    pub_rate = rospy.get_param('~pub_rate', 0.3)
    rospy.loginfo('[+] object detection node is running at %d Hz'%int(1/pub_rate))
    timer = rospy.Timer(rospy.Duration(pub_rate), timer_callback)
    rospy.spin()
    