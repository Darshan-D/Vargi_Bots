# Vargi_Bots
Documentation and implementation of eYRC 2020-2021 Vargi Bots theme

The aim of this theme is to automate the whole warehouse, where a robotic arm will dispatch and ship the incoming order using conveyor belt.

# Concepts Used

1. Robotic Operating System (ROS)
2. Robotic Manipulation
3. Robotic Perception
4. Internet of Things (IoT)
5. Google App Scripting
6. Javascript
7. Python


Introduction
============

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

So how it works? / What is the flow?
------------------------------------

As soon as an order is received by the warehouse a camera looks at 
the available packages on the shelf, finds the position of the
required package on the shelf and sends that position to a robotic 
arm. The robotic arm used in this warehouse is UR5. The arm goes to
the given position and grabs the package using vaccum suction, and 
places it on the conveyor belt. There is another UR5 robotic arm on
the other end of the conveyor belt. This arm picks up the package and
puts it inside a bin, there are 3 diffrent bins, each representing 3
different priority orders. This arm detects the package with the help
of yet another camera this time present on top of the conveyor belt

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

Here is our implementation Video of the same:
https://www.youtube.com/watch?v=dQw4w9WgXcQ
