# F1tenth
Code repo for F1tenth.
All the cpp and python files need to be run under the simulator

# Abstract
Repo which contains lab code for F1tenth and python code of master research on jupyter notebook

# ROS install
Please follow the installing guide here ***http://wiki.ros.org/melodic/Installation/Ubuntu***

# F1tenth simulator setup
Please follow the installing guide here ***https://f1tenth.org/build.html***
All the software simulation is based on this simulator.

## Simulator dependencies

install the following dependencies using the command
```
sudo apt-get install ros-melodic-tf2-geometry-msgs ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-map-server
```
install the simulator package
```
cd ~/catkin_ws/src
git clone https://github.com/f1tenth/f1tenth_simulator.git
```
Then run catkin_make to build it
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
Finally launch the simulator with the following command
```
roslaunch f1tenth_simulator simulator.launch
```
# Running lab code in the simulator
Please follow the lab 1 instructions on ***https://f1tenth.org/learn.html*** to get familiar with ROS communication system.
Please download and configure the CMakeLists.txt file while working on the corresponding labs

# Jupyter notebook

## install jupyter notebook
If you want to run .ipynb file located on Master research project/jupyter notebook folder. Please install Anaconda first ***https://www.anaconda.com/products/distribution***, you can install jupyter notebook inside Anaconda navigator.

## install CVXPY package
.ipynb file requries installing external package CVXPY on ***https://www.cvxpy.org/install/***. Installing CVXPY using pip command

```
pip install cvxpy
```



