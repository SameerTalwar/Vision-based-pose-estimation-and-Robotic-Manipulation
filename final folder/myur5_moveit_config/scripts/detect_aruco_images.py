#!/usr/bin/env python
'''
Sample Command:-
python detect_aruco_images.py --image Images/test_image_1.png --type DICT_5X5_100
'''
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from utils import ARUCO_DICT, aruco_display
import argparse


ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

# verify that the supplied ArUCo tag exists and is supported by OpenCV
if ARUCO_DICT.get(args["type"], None) is None:
	print("ArUCo tag type '{args[type]}' is not supported")
	sys.exit(0)

class image_converter:

  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/myur5/camera1/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.imshow("Image window", cv_image)

    #print("Loading image...")
    #h,w,_ = cv_image.shape
    #width=600
    #height = int(width*(h/w))
    #image = cv2.resize(image, (width, height), interpolation=cv2.INTER_CUBIC)

    # load the ArUCo dictionary, grab the ArUCo parameters, and detect
    # the markers
    print("Detecting '{}' tags....".format(args["type"]))
    arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
    arucoParams = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, arucoDict, parameters=arucoParams)

    detected_markers = aruco_display(corners, ids, rejected, cv_image)
    cv2.imshow("Image", detected_markers)

    # # Uncomment to save
    # cv2.imwrite("output_sample.png",detected_markers)

    cv2.waitKey(0)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)