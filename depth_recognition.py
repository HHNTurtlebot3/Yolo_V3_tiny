#!/usr/bin/env python
import rospy
import cv2
import time
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header
from std_msgs.msg import String
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

depth_array = []

def convert_depth_image(ros_image):		#https://github.com/IntelRealSense/realsense-ros/issues/714
    global depth_array
    bridge = CvBridge()
     # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
     #Convert the depth image using the default passthrough encoding
        depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)
        #print("Get depth_image.")
        #center_idx = np.array(depth_array.shape) / 2
        #print('center depth:', depth_array[center_idx[0], center_idx[1]]) #in mm

    except CvBridgeError as e:
        print (e)
     #Convert the depth image to a Numpy array

def callback(data):
    for box in data.bounding_boxes:
        a= []
        for i in range(box.xmin, box.xmax): #schauen python box zu box andere funktion fuer i und j
            for j in range(box.ymin, box.ymax):
                if depth_array[j,i] != 0:
                    a.append(depth_array[j,i])
        print("BoundingBox_ID: {}, BoundingBox_Class: {}, Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}, Depth_min: {}".format(box.id, box.Class, box.xmin, box.xmax, box.ymin, box.ymax, min(a))) #tiefenpixel als mm
        #minimum = min(a)
        #print("Minimale distance is:", min)

rospy.init_node('pixel2depth',anonymous=True)
rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image,convert_depth_image, queue_size=1)
rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , callback, queue_size=1)
#rospy.Publisher('/min_distance')

if __name__ == '__main__':
    rospy.spin()
