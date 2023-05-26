# Jupyter Notebook User Interface for Mobile Robot Simulator 

## Introduction
Jupyter Notebook is an open source web application which is used to create and share documents that contain real-time code, equation, data visualizations, text, and so on. This Jupyetr Nootebook is designed to create the User Interface of my code. Also, this notebook is developed as an user interface for the the project Software Architecture for Mobile Robot Control. For this, Jupyter Notebook tool is used.

This user interface should be able to control the project Mobile Robote Simulator. Also, user can operates and change the robot's behvaiour depending on their choice. Robot's Behvaiour can be autonomous drive, manual drive using teleop, and manual drive using teleop and avoiding the collisions.

## Installation
To create Jupyter Interface, Jupyter Notebook tool is required on your system. To install Jupyter follow the steps given below:

```
pip3 install jupyter bqplot pyyaml ipywidgets
```

```
jupyter nbextension enable --py widgetsnbextension
```
To get started, all you need to do is open up your terminal application and go to a folder of your choice. Then run the below command:
```
jupyter notebook --allow-root
```

Below is the figure which shows Notebook Server.
![jupyter](https://user-images.githubusercontent.com/17598805/230768480-cb618e51-020c-4788-9d5c-5c556704e58e.PNG)

## Jupyter and ROS

Jupyter Notebooks may be used and integrated with ROS called as [Jupyter-Ros](https://jupyter-ros.readthedocs.io/en/latest/). As for the other libraries, we need to install some extensions: 
```
pip3 install jupyros
```
For the publishing, the package contains tools to automatically generate widgets from message definitions. 
```
import jupyros as jr
import rospy
from std_msgs.msg import String
rospy.init_node('jupyter_node')
jr.publish('/sometopic', String)
```
This results in a jupyter widget where one can insert the desired message in the text field. The form fields (jupyter widgets) are generated automatically from the message definition. If we use a a different message type, we will get different fields.

**ROS3D** communicates with ROS via websocket. This communication is configured through the jupyter widgets protocol, but you are also required to run the *“rosbridge websocket”* package in your ROS environment (or launch file). For this, you need to make sure that you have ros-noetic-rosbridge-suite and ros-noetic-tf2-webrepublisher. Thus, for this example, install:
```
apt-get install ros-noetic-rosbridge-suit
```
```
apt-get install ros-noetic-tf2-web-republisher
```
```
apt-get install ros-noetic-slam-gmapping
```
```
apt-get install ros-noetic-move-base
```
For non-Docker Image user execute the aforementioned command by adding ***sudo*** in front. 

## Description of the Jupyter Code

The Jupyter user interface node is a super easy node. This is used to control the robot's behaviour. This implementation will demonstrates that robot's behavior such as switching to the different modalities such as:

* Autonomously reach a x,y coordinate inserted by the user
* Letting the user drive the robot with the keyboard
* Letting the user drive the robot assisting them to avoid collisions 

Additionally, these modalities can also be managed by using this interface. 
