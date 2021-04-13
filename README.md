# Vargi_Bots
Documentation and implementation of eYRC 2020-2021 Vargi Bots theme

The aim of this theme is to automate the whole warehouse, where a robotic arm will dispatch and ship the incoming order, buyers will receive the e-mail alerts, and the current status of the inventory will be updated live on the warehouse website.

A special thanks my team members in the competition, without whom the whole project might not have been possible
* Rutuja Choudary
* Prajyoth Poonja
* Mrudula Acharya

Here is our implementation Video of the same:
https://youtu.be/sZMcY_kCU2w

## Concepts Used

1. Robotic Operating System (ROS)
2. Gazebo
3. Move it!
4. Robotic Manipulation
5. Robotic Perception
6. MQTT Protocol (For IoT Applications)
7. Google App Scripting
8. Javascript
9. Python
10. OpenCV


## Introduction

We all have ordered something online, for us it arrives at a click
of a button. But behind the scenes, there is a whole complex
logistics chain compromising of many people, who work day in
and night out to deliver the item you ordered. As humans we are
prone to mistakes and cannot work with maximum efficiency all the
time. So what if, all these was automated?

Compared to humans, robots can work at maximum efficiency all the
time, and are less prone to mistakes. So working on the same idea 
here we present to you a completely automated warehouse. As a strong
belivers in Industrial Revolution 4.0, everything in this warehouse
is automated!

### So how it works? / What is the flow?

As soon as an order is received by the warehouse a camera looks at 
the available packages on the shelf, finds the position of the
required package on the shelf and sends that position to a robotic 
arm. The robotic arm used in this warehouse is UR5. Using inverse 
kinematics, the arm goes to the given position and grabs the package 
using vaccum suction, and places the package on the conveyor belt. 
There is another UR5 robotic arm on the other end of the conveyor belt. 
This arm picks up the package and puts it inside a bin. There are 3 
diffrent bins, each representing 3 different priority orders. 
Second arm detects the package with the help of yet another camera this
time present on top of the conveyor belt

If those too many sentences confused you here is a simplified bullet 
points version of the above paragraph.

1. Order is Received by the Warehouse
2. A camera scans for available packages on the shelf
3. Camera finds position of the required order (package) on the shelf
4. A robotic arm moves to the position of package, given by camera
5. Robotic arm grabs the package and places it on a moving conveyor belt
6. Another camera present on the other side of the conveyor belt looks for that package
7. After detecting that package, it triggers the second robotic arm which is also present on the other side of the conveyor.
8. The second robotic arm grabs the package from the moving conveyor and places it inside a bin, from where it can be loaded and delivered.

This is sort of a brief summary of what we have done.

## ROS Packages Developed
There are two ROS packages developed by us, they are as follows:

* pkg_task5
* pkg_ros_iot_bridge

### pkg_task5
The package was developed in task5 of the eYRC competition hence the name. The objective of this package is to control the two UR5 Arm present in the Gazebo using Moveit API's. Moreover, all the Computer Vision tasks like finding the required order (package) from the shelf and detecting the package on the conveyor belt is done by this package

### pkg_ros_iot_bridge
This package is used to upload the data on the google spreadsheet, the inventory website, and send the alert e-mails to the buyers.
