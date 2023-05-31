#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# intialize moveit_commander and rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)

#instantiate a RobotCommander object
robot = moveit_commander.RobotCommander()

#instantiate a PlanningSceneInterface Object
scene = moveit_commander.PlanningSceneInterface()

#instantiate a MoveGroupCommander Object
group_name="manipulator"
move_group=moveit_commander.MoveGroupCommander(group_name)

#create displayTrajectory ROS publisher which is used to display trajectories in RVIZ
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

#Get basic info
planning_frame = move_group.get_planning_frame()
print('Planning frame: %s' % planning_frame)

#printing name of end effector
eef_link = move_group.get_end_effector_link()
print("=========== End effector link: %s" % eef_link)

#we can get list of all grps in the robot:
group_names = robot.get_group_names()
print("=========== Available planning groups:", robot.get_group_names())

#printing entire state of robot:
print("============ Prinitng Robot state")
print(robot.get_current_state())
print("============")

#getting joint values from grp
joint_current = move_group.get_current_joint_values()

print('Current joint values: ', joint_current)
print('Enter joint angles in radian')
joint_goal = [(input("Enter_angle: ")) for i in range(6)]
print('New goals for the robot:', joint_goal)
print((pi - joint_goal[2] - joint_goal[1]))
if (pi + abs(joint_goal[1]) - abs(joint_goal[2])) < pi:
    joint_goal[3] = (pi + abs(joint_goal[1]) - abs(joint_goal[2]))
else : joint_goal[3] = (pi + abs(joint_goal[1]) - abs(joint_goal[2])) - (2*pi)
print(joint_goal[3])
joint_goal[4] = -(pi/2)

#increasing planning time
#move_group.setPlanningTime(10)

# go command can be directly called after setting pose or joint target
move_group.go(joint_goal, wait=True)

#calling ''stop()'' ensures no residual movement
move_group.stop()