

## Installation and Setup

The simulator itself depends on libraries, packages and objects from ROS and Gazebo. The current version of the simulator uses ROS Noetic and Gazebo 11; along with certain other libraries and their python bindings. Before proceeding with the installation steps in this document, check if your local machine already has the full-desktop version of ROS Noeticinstalled. If not, follow the instructions provided in this [link](http://wiki.ros.org/noetic/Installation/Ubuntu). When you get to section 1.4 of the ROS installation tutorial, choose the first option: `ros-noetic-desktop-full`. Your local machine will now have the basic dependencies installed. The next steps of the installation tutorial are critical and must be followed in the shown sequence.

First, we begin by installing the ROS and Gazebo packages that are not installed by default using the above method. This includes SLAM packages, ROS navigation.

```
sudo apt-get install -y ros-noetic-navigation ros-noetic-teb-local-planner* ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-control ros-noetic-ackermann-msgs ros-noetic-serial 
```

Then, we build this catkin package and source the workspace. Once it is built, the models have to be placed in the appropriate directory for gazebo to find them. 
Your home folder should have a ```.gazebo``` directory, you can check this with 
```
ls -lA | grep .gazebo   
```
if this command does not return anything then create one as follows
```
cd
mkdir -p .gazebo/models
```
From the audubon_gazebo directory execute 
```
 cp -r world/models/. ~/.gazebo/models
```
This will move the models to the appropriate directory

You can verify that using the ls command in the ~/.gazebo/models directory

## Usage
Once the package is built and the models are in the appropriate directory, (source the workspace once it is built) run the testing ground world using 
```
roslaunch audubon-gazebo testing_ground.launch 
```
This should launch an empty plane with the car. 
To verify if everything else works use the other provides launch files 

To verify that the car works, publish a ackerman message using rostopic pub to the topic /car1/command (populate only the steering angle and speed should be fine. The rest can be zero or unpopulated) 

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
