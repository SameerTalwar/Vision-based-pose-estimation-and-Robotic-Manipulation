#!/usr/bin/env python
import sys, time
import numpy as np
from scipy.ndimage import filters
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage

class image_read:
    def __init__(self):
        # Define subscribed topic
        self.subscriber = rospy.Subscriber("/myur5/camera1/rgb/image_raw/compressed", CompressedImage, self.callback, queue_size=1)

    def callback(self, ros_data):

        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        resized_image = cv2.resize(image_np, (320,240), interpolation=cv2.INTER_NEAREST)
        cv2.imshow('cv_img', resized_image)
        cv2.waitKey(2)

def main(args):
    ic=image_read()
    rospy.init_node('image_read', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down the ROS  image reader node')
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)