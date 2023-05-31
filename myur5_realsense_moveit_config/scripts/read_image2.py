#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import glob

####---------------------- CALIBRATION ---------------------------
# termination criteria for the iterative algorithm
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# checkerboard of size (7 x 6) is used
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# iterating through all calibration images
# in the folder
images = glob.glob('calib_images/checkerboard/*.jpg')
cardboardfound=False

for fname in images:
  cardboardfound=True
  img = cv2.imread(fname)
  gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

  # find the chess board (calibration pattern) corners
  ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

  # if calibration pattern is found, add object points,
  # image points (after refining them)
  if ret == True:
    objpoints.append(objp)

    # Refine the corners of the detected corners
    corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    imgpoints.append(corners2)

    # Draw and display the corners
    img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)

#if (cardboardfound):
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

class image_converter:

  def __init__(self):
    self.marea_pub = rospy.Publisher("/Marker_Area", Float64, queue_size=10)
    self.erry_pub = rospy.Publisher("/errorY", Float64, queue_size=10)
    self.errz_pub = rospy.Publisher("/errorZ", Float64, queue_size=10)
    self.rotvec_pub = rospy.Publisher("/rotationVector", String, queue_size=10)
    self.transvec_pub = rospy.Publisher("/translationVector", String, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/realsensecamera/color/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    #Load the dictionary that was used to generate the markers.
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)

    # Initialize the detector parameters using default values
    parameters =  cv2.aruco.DetectorParameters_create()

    # Detect the markers in the image
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(cv_image, dictionary, parameters=parameters)

    detected_markers = cv2.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds)
    cv2.imshow("Image", detected_markers)

    #Choose marker ID
    ID = 4
    if (markerIds is not None):
      self.rvec, self.tvec ,_ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.05, mtx, dist)
      '''print("rvec,tvec",self.rvec,self.tvec)
      for i in range(0, markerIds.size):
        # draw axis for the aruco markers
        cv2.drawFrameAxes(cv_image, mtx, dist, self.rvec[i], self.tvec[i], 0.05)'''
      for IDindex in range(len(markerIds)):
        if (markerIds[IDindex]==ID):
          cv2.drawFrameAxes(cv_image, mtx, dist, self.rvec[IDindex], self.tvec[IDindex], 0.05)
          print("rvec,tvec",self.rvec[IDindex],self.tvec[IDindex])
          rotv1 = self.rvec[IDindex][0][0]
          rotv2 = self.rvec[IDindex][0][1]
          rotv3 = self.rvec[IDindex][0][2]
          sr = [rotv1,rotv2, rotv3]
          rlistToStr = ' '.join([str(elem) for elem in sr])
          self.rotvec_pub.publish(rlistToStr)
          transv1 = self.tvec[IDindex][0][0]
          transv2 = self.tvec[IDindex][0][1]
          transv3 = self.tvec[IDindex][0][2]
          st = [transv1,transv2,transv3]
          tlistToStr = ' '.join([str(elem) for elem in st])
          self.transvec_pub.publish(tlistToStr)
          #break
          positions = markerCorners[IDindex]
          print("positions", positions[0])
          self.centroid = positions.mean(axis=1)
          self.erry = (400 - self.centroid[0][0])
          self.erry_pub.publish(self.erry)
          self.errz = (400 - self.centroid[0][1])
          self.errz_pub.publish(self.errz)
          print("publishing erry,errz", self.erry, self.errz)
          #print(self.centroid)
          cv2.circle(cv_image, (self.centroid[0][0],self.centroid[0][1]), 5, (0,0,255), -1)
          n = len(positions[0]) # of corners
          print(n)
          Marea = 0.0
          for i in range(n):
              j = (i + 1) % n
              #print(positions[0][i][0],positions[0][j][1],positions[0][j][0],positions[0][i][1])
              Marea += positions[0][i][0] * positions[0][j][1]
              Marea -= positions[0][j][0] * positions[0][i][1]
          Marea = abs(Marea) / 2.0
          print("publishing Marker area:", Marea)
          self.marea_pub.publish(Marea)

    detected_markers = cv2.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds)
    cv2.imshow("Image", detected_markers)

    cv2.waitKey(3)

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