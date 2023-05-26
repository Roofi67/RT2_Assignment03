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
First, imported the required libraries as given below:
```
import rospy
import time
import numpy as np
import ipywidgets as widgets
from IPython.display import display
from ipywidgets import Button, Layout
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import transformations
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from matplotlib.animation import FuncAnimation
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalStatusArray
from IPython.display import clear_output
```
## Modalities management code.
In this section, after init the jupyter_ui node, I decided to subscribe to the LaserScan topic to get all the informations for avoiding the walls for third modality.
```
rospy.init_node("jupyter_ui")
output = widgets.Output()

des_x = widgets.FloatText(
    value=0,
    description='X:',
    disabled=False
)
des_y = widgets.FloatText(
    value=0,
    description='Y:',
    disabled=False
)

boxes = widgets.HBox([des_x, des_y])

cancel_button = widgets.Button(description = 'Cancel the goal!', layout=Layout(width='50%', height='80px'), button_style='danger')

def cancel_button_clicked(b):
    rospy.set_param('active', 0)
    
    
cancel_button.on_click(cancel_button_clicked)
    
ok_button = widgets.Button(description = 'Send Goal', layout=Layout(width='30%', height='30px'), button_style='')

def ok_button_clicked(b):

    rospy.set_param('active', 0)
    rospy.set_param('des_pos_x', des_x.value)
    rospy.set_param('des_pos_y', des_y.value)
    rospy.set_param('active', 1) 
    
ok_button.on_click(ok_button_clicked)
    
button_first_mod = widgets.Button(description = 'First modality, insert goal coordinates!', layout=Layout(width='50%', height='80px'))
button_first_mod.style.button_color = 'blue'

button_second_mod = widgets.Button(description = 'Second modality, teleop your robot!',
                                   layout=Layout(width='50%', height='80px'))
button_second_mod.style.button_color = 'lightblue'

button_third_mod = widgets.Button(description = 'Third modality, teleop your robot with assistance!', layout=Layout(width='50%', height='80px'), button_color='salmon')
button_third_mod.style.button_color = 'green'


def first_button_clicked(b):
    back_to_menu_mod()
    print("Target Goal based first modality")
    third = False
    display(des_x)
    display(des_y)
    display(ok_button)
    display(cancel_button)
    
button_first_mod.on_click(first_button_clicked)

def second_button_clicked(b):
    back_to_menu_mod()
    third = False
    rospy.set_param('active', 0)
    rospy.set_param('active', 2)
    display(row1)
    display(row3)
    display(row4)
    display(row5)
    
    
button_second_mod.on_click(second_button_clicked)

def clbk_laser(msg):
    global vel
    global ok_left
    global ok_right
    global ok_straight
    global publ

    right = min(min(msg.ranges[0:143]), 1)      # Detects obstacles on the right of the robot
    front = min(min(msg.ranges[288:431]), 1)    # Detects obstacles in front of the robot
    left = min(min(msg.ranges[576:719]), 1)     # Detects obstacles on the left of the robot

    if right != 1.0 and vel.angular.z < 0:
        ok_right =False
        vel.angular.z = 0

    else:         
        ok_right =True

    if front != 1.0 and vel.linear.x > 0:  
        ok_straight = False
        vel.linear.x = 0

    else:              
        ok_straight =True

    if left != 1.0 and vel.angular.z > 0:    
        ok_left =False
        vel.angular.z = 0

    else:            
        ok_left =True
        
    publ.publish(vel)

def third_button_clicked(b):
    back_to_menu_mod()
    third=True
    rospy.set_param('active', 0)
    rospy.set_param('active', 3)
    display(row1)
    display(row3)
    display(row4)
    display(row5)
    
button_third_mod.on_click(third_button_clicked)

def back_to_menu_mod():
    clear_output(wait=True)
    display(buttons)

sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
buttons = widgets.HBox([button_first_mod, button_second_mod, button_third_mod])
display(buttons)
```
## Graphical Representation
Here is the implementation of the graphs for showing the data of the robot. There are three graphs:

### Position of the robot using LaserScan data.
### current position of the robot using Radar vision.
### Reaching the Goal status of the robot.

![laser_data](https://github.com/Roofi67/RT2_Assignment03/assets/95746070/217fc85a-54f8-4592-81ad-246cf6d6a07d)

