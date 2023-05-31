#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import roslib
import cv2
import math
import numpy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Float64
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class MoveItContext(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv) # intialize moveit_commander and rospy node
        rospy.init_node('robotic_arm_moveit', anonymous=True)

        robot = moveit_commander.RobotCommander() #instantiate a RobotCommander object
        scene = moveit_commander.PlanningSceneInterface() #instantiate a PlanningSceneInterface Object

        group_name = "manipulator" #instantiate a MoveGroupCommander Object
        move_group = moveit_commander.MoveGroupCommander(group_name)

        #create displayTrajectory ROS publisher which is used to display trajectories in RVIZ
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        planning_frame = move_group.get_planning_frame() #Get basic info
        end_effector_link = move_group.get_end_effector_link() #printing name of end effector
        group_names = robot.get_group_names() #we can get list of all grps in the robot:

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.end_effector_link = end_effector_link
        self.group_names = group_names
        self.centroid = [[400, 400]]
        self.erry = (400 - self.centroid[0][0])
        self.errz = (400 - self.centroid[0][1])
        self.atHome = False
        self.Marea = 0.0
        self.rvec = [0.0, 0.0, 0.0]
        self.tvec = [0.0, 0.0, 0.0]

        #self.image_pub = rospy.Publisher("image_topic_2",Image)

        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("/myur5/camera1/rgb/image_raw",Image,self.callback)
        self.erry_sub = rospy.Subscriber("errorY",Float64,self.ERRYcallback)
        self.errz_sub = rospy.Subscriber("errorZ",Float64,self.ERRZcallback)
        self.rvec_sub = rospy.Subscriber("rotationVector",String,self.rveccallback)
        self.tvec_sub = rospy.Subscriber("translationVector",numpy_msg(Floats),self.tveccallback)
        self.marea_sub = rospy.Subscriber("Marker_Area",Float64,self.Mareacallback)

    def Mareacallback(self,data):
        self.Marea = float(data.data)
        #print("subscribing to Marea", self.Marea)
        
        Marea = self.Marea

        if (abs(self.erry)+abs(self.errz)+abs(1-(Marea/90000.0))>1)&(self.atHome):
            print("Starting planning to go to pose goal.\n")

            current_pose = self.move_group.get_current_pose().pose
            #print(erry,errz)
            print("Marea error", 0.15*(1-(Marea/90000.0)), "yerr",0.0012*self.erry,"zerr", 0.0012*self.errz)
            pose_goal = geometry_msgs.msg.Pose()
            
            print(self.rvec)
            roll_angle = self.rvec[0]
            pitch_angle = self.rvec[1]
            yaw_angle = self.rvec[2]
            # Pose Orientation
            quaternion = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)
            print("quat", quaternion)

            pose_goal.orientation.x = current_pose.orientation.x #+ 1 - quaternion[0]
            pose_goal.orientation.y = current_pose.orientation.y #+ quaternion[1]
            pose_goal.orientation.z = current_pose.orientation.z #+ quaternion[2]
            pose_goal.orientation.w = current_pose.orientation.w #+ quaternion[3]

            pose_goal.position.x = current_pose.position.x + 0.02*(1-(Marea/90000.0))
            pose_goal.position.y = current_pose.position.y + 0.0008*self.erry
            pose_goal.position.z = current_pose.position.z + 0.0015*self.errz

            self.move_group.set_pose_target(pose_goal)

            plan = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            print("ended planning")

        #else:
        #    print("bypassing")
        #if (self.atHome):
        #    MoveItContext().go_to_defined_joint()

    def ERRYcallback(self,data):
        self.erry = float(data.data)
        #print("subscribing to erry", self.erry)

    def ERRZcallback(self,data):
        self.errz = float(data.data)
        #print("subscribing to errz", self.errz)

    def rveccallback(self,data):
        StrList = list(data.data.split(" "))
        self.rvec = list(numpy.float_(StrList))
        #print("subscribing to rvec", self.rvec)
    
    def tveccallback(self,data):
        StrList = list(data.data.split(" "))
        self.tvec = list(numpy.float_(StrList))
        #print("subscribing to tvec", self.tvec)


    def go_to_home_pose_goal(self):

        print("Starting planning to go to home pose goal.\n")
        current_pose = self.move_group.get_current_pose().pose

        home_pose_goal = geometry_msgs.msg.Pose()
        home_pose_goal.orientation.w = 0.4999854087
        home_pose_goal.orientation.x = -0.500609492
        home_pose_goal.orientation.y = 0.5000431720
        home_pose_goal.orientation.z = -0.499910456
        home_pose_goal.position.x = 0.297741609641
        home_pose_goal.position.y = -0.109139008406
        home_pose_goal.position.z = 1.57022492418

        self.move_group.set_pose_target(home_pose_goal)

        plan = self.move_group.go(wait=True)
        self.move_group.stop()

        self.move_group.clear_pose_targets()

        print(self.centroid)
        print("ended planning")
        self.atHome = True

        # For testing:
        current_pose = self.move_group.get_current_pose().pose
        return all_close(home_pose_goal, current_pose, 0.1)

    def get_current_pose(self):
        current_pose = self.move_group.get_current_pose().pose
        print(current_pose)

    def go_to_defined_joint(self):

        #getting joint values from grp
        joint_current = self.move_group.get_current_joint_values()
        print('Current joint values: ', joint_current)
        print(self.rvec[2])
        
        #defining goal joint angles
        joint_goal = joint_current

        if (abs(self.rvec[2])>0.05):
            joint_goal[4] = joint_current[4] +0.3*self.rvec[2]
            if (joint_goal[4]>pi):
                joint_goal[4]=joint_goal[4]-(2*pi)
            elif (joint_goal[4]<-pi):
                joint_goal[4]=joint_goal[4]+(2*pi)
        if (abs(self.rvec[1])>0.05):
            joint_goal[5] = joint_current[5] +0.3*self.rvec[1]
            if (joint_goal[5]>pi):
                joint_goal[5]=joint_goal[5]-(2*pi)
            elif (joint_goal[5]<-pi):
                joint_goal[5]=joint_goal[5]+(2*pi)
        if (abs(3.0 - self.rvec[0])>0.05):
            joint_goal[3] = joint_current[3] -0.07*(3.0 - self.rvec[0])
            if (joint_goal[3]>pi):
                joint_goal[3]=joint_goal[3]-(2*pi)
            elif (joint_goal[3]<-pi):
                joint_goal[3]=joint_goal[3]+(2*pi)

        print("Starting planning to go to set joints.\n")
        # go command can be directly called after setting pose or joint target
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        print("ending planning")

    def go_to_home_joint(self):

        #getting joint values from grp
        joint_current = self.move_group.get_current_joint_values()
        print('Current joint values: ', joint_current)
        joint_goal=joint_current
        joint_goal[0] = 0
        joint_goal[1] = -2.4
        joint_goal[2] = 1.8
        if (pi + abs(joint_goal[1]) - abs(joint_goal[2])) < pi:
            joint_goal[3] = (pi + abs(joint_goal[1]) - abs(joint_goal[2]))
        else : joint_goal[3] = (pi + abs(joint_goal[1]) - abs(joint_goal[2])) - (2*pi)
        print(joint_goal[3])
        joint_goal[4] = -(pi/2)
        joint_goal[5] = 0

        print("Starting planning to go to set joints.\n")
        # go command can be directly called after setting pose or joint target
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        print("ending planning")
        self.atHome = True


def main():
    moveit_context = MoveItContext()
    moveit_context.get_current_pose()
    #moveit_context.go_to_home_joint()
    moveit_context.go_to_home_pose_goal()
    try:
        #moveit_context.get_current_pose()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass