

## Installation and Setup

The simulator itself depends on libraries, packages and objects from ROS and Gazebo. The current version of the simulator uses ROS Noetic and Gazebo 11; along with certain other libraries and their python bindings. Before proceeding with the installation steps in this document, check if your local machine already has the full-desktop version of ROS Noeticinstalled. If not, follow the instructions provided in this [link](http://wiki.ros.org/noetic/Installation/Ubuntu). When you get to section 1.4 of the ROS installation tutorial, choose the first option: `ros-noetic-desktop-full`. Your local machine will now have the basic dependencies installed. The next steps of the installation tutorial are critical and must be followed in the shown sequence.

First, we begin by installing the ROS and Gazebo packages that are not installed by default using the above method. This includes SLAM packages, ROS navigation.

```
sudo apt-get install -y ros-noetic-navigation ros-noetic-teb-local-planner* ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-control ros-noetic-ackermann-msgs ros-noetic-serial 
```

Based on the work of:
```
@inproceedings{babu2020f1tenth,
  title={f1tenth. dev-An Open-source ROS based F1/10 Autonomous Racing Simulator},
  author={Babu, Varundev Suresh and Behl, Madhur},
  booktitle={2020 IEEE 16th International Conference on Automation Science and Engineering (CASE)},
  pages={1614--1620},
  year={2020},
  organization={IEEE}
}
```
