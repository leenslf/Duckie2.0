#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

class Uncompressor:
    def __init__(self):
        rospy.init_node('py_republish', anonymous=True)
        self.bridge = CvBridge()
        
        # Subscriber: Listens to the compressed topic from the Bag
        # NOTE: We subscribe to the full name directly
        self.sub = rospy.Subscriber(
            "/horse/camera_node/image/compressed", 
            CompressedImage, 
            self.callback, 
            queue_size=1
        )
        
        # Publisher: Sends raw images to VINS
        self.pub = rospy.Publisher(
            "/horse/camera_node/image/raw", 
            Image, 
            queue_size=1
        )
        
        print("Python Uncompressor Started! Waiting for bag...")

    def callback(self, msg):
        try:
            # 1. Unzip the JPEG
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE) # Force grayscale for VINS
            
            # 2. Check Resolution (Optional Debug)
            # print(f"Image shape: {cv_image.shape}") 

            # 3. Publish as Raw Image
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="mono8")
            img_msg.header = msg.header # Keep the timestamp!
            self.pub.publish(img_msg)
            
        except Exception as e:
            print(e)

if __name__ == '__main__':
    node = Uncompressor()
    rospy.spin()