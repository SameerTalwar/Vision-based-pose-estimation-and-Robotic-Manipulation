# Follow this procedure to run the package 
#Supported Ubuntu and ROS versions- ROS Melodic, Ubuntu 18.04
---------------------------------------------------------------------------------
# Install universal_robot package
#Clone the following git packages into catkin_ws/src

cd $HOME/catkin_ws/src

#retrieve the sources (replace '$ROS_DISTRO' with the ROS version you are using)
git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git

cd $HOME/catkin_ws

#checking dependencies (again: replace '$ROS_DISTRO' with the ROS version you are using)
rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

#building
catkin_make

#activate this workspace
source $HOME/catkin_ws/devel/setup.bash

#or.. installing directly using sudo apt
sudo apt-get install ros-$ROS_DISTRO-universal-robot

#You can also copy the package directly into catkin_ws/src and repeat from the rosdep step

----------------------------------------------------------------------------------
# Next install the moveit package

#make sure you have the most up to date packages:
rosdep update
sudo apt-get update
sudo apt-get dist-upgrade

sudo apt-get install ros-melodic-moveit

#Source installation requires wstool, catkin_tools, and optionally clang-format:
sudo apt install python-wstool python-catkin-tools clang-format-10 python-rosdep

#In catkin_ws-
cd catkin_ws
source /opt/ros/melodic/setup.bash

#Pull down required repositories and build from within the root directory of your catkin workspace
wstool init src
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release

-----------------------------------------------------------------------------------

# copy the folder myur5_sim in catkin_ws/src

#run a catkin_make
catkin_make

#In case of any missing package error while executing catkin_make, simply install it using-
sudo apt-get install <package name>

------------------------------------------------------------------------------------
  
# In order to run the gazebo world
roslaunch myur5_description myur5.launch

#in separate terminal, run the move_group python node, this also opens rviz with motion planner
roslaunch myur5_moveit_config myur5_move_group_python.launch

#run the python scripts to move the arm and go to an aruco marker
rosrun myur5_moveit_config move_group_python_interface2.py

#you can specify the desired id of the aruco marker that you want to go to-
#in file myur5_moveit_config/scripts/move_group_python_interface2.py, go to line 91
#in the MarkerId variable, specify the ID {1,2,3,4,12 or 13}

#if there is an error while running python script files, go to the file properties and set the python files as executable via program

----------------------------------------------------------------------------------------

# Alternate approach to aruco detection and tracking-
#in separate terminals run the following commands

roslaunch myur5_description myur5.launch
roslaunch myur5_moveit_config myur5_move_group_python.launch
rosrun myur5_moveit_config read_images.py
rosrun myur5_moveit_config move_group_python_interface3.py

------------------------------------------------------------------------------------------

# Pose estimation on aruco markers, refer to the following-
#https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
#https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d

python move_group_python_interface4.py
#file to move arm according to pose of aruco markers


---------------------------------------------------------------------------------------
# new folders added- doing the same thing as before, but while using intel realsense D435 as camera.

#pose correction by running the following two scripts instead of before
rosrun myur5_realsense_moveit_config read_images2.py
rosrun myur5_realsense_moveit_config move_group_python_interface4.py


