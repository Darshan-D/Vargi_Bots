#!/usr/bin/env python

"""
"node_package_info" is used to do all the image processing tasks.
This node will scan all the packages and save the pose and priority of
the package as a key value pair, and send it other ROS nodes.
"""

import rospy
import cv2
import sys
import actionlib
import threading

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from pyzbar.pyzbar import decode

from std_msgs.msg import String

from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal
from pkg_ros_iot_bridge.msg import msgRosIotResult

class CameraActionClient:
  """
  This class is used for image processing from 2D Camera

  This function will initialize this node as ROS Action Client
  This will also publish all the package_info and package_pose 
  to other ROS nodes 
  """

  def __init__(self):
    """
    The Constructor for CameraActionClient.

    It sets up the bridge between openCV and ROS. It will also, setup
    this node as an publisher to "/eyrc/vb/camera_1/image_raw" ROS topic.
    """

    self.bridge = CvBridge()

    # Subscribing to ROS "/eyrc/vb/camera_1/image_raw" topic to get an input image
    self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)
    
    # Creating a handle to publish the messages to a topic
    self.pub_handle = rospy.Publisher('/pkg_task5/package_info', String, queue_size = 10)
    
    # Dictionary to store, the package name and color as a key value pair
    self.package_pose_color = {"packagen00": "NA",
                              "packagen01": "NA",
                              "packagen02": "NA",
                              "packagen10": "NA",
                              "packagen11": "NA",
                              "packagen12": "NA",
                              "packagen20": "NA",
                              "packagen21": "NA",
                              "packagen22": "NA",
                              "packagen30": "NA",
                              "packagen31": "NA",
                              "packagen32": "NA"}

    self.all_info_aquired = False
    self.packages_updated_on_gsheet = 0

    # Making it action client
    self._ac = actionlib.ActionClient('/action_ros_iot',
                                      msgRosIotAction)

    # Dictionary to store the goal handles
    self._goal_handles = {}

    # Wait for the action server to start
    rospy.loginfo("[CAMERA CLIENT] Waiting for Action server ...")
    self._ac.wait_for_server()


  def pre_process_img(self, arg_image):
    """
    This function is used to do preprocess the incoming image

    Preprocessing includes converting to gray scale, bluring and
    thresholding the image. All these helps us to read the different
    QR codes from the image

    Parameters:
      arg_image (image): The input list (image) from the 2D Camera

    Returns:
      thresh_im (image): The preprocessed version of the argument image
    """

    # Converting image to grey scale
    grey_image = cv2.cvtColor(arg_image, cv2.COLOR_BGR2GRAY)

    # Bluring the image
    blur_im = cv2.GaussianBlur(grey_image, (3,3), 0)

    # Doing binarization
    thresh_im = cv2.adaptiveThreshold(blur_im,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,11,2)
    
    # cv2.imshow('Thresh', cv2.resize(thresh_im, (720/2, 1280/2)))

    return thresh_im

  def get_qr_data(self, arg_image):
    """
    This function is used to read the QR codes from the image.

    After reading the QR data, it stores them as value to the pacakge name
    which is used as key. This helps in figuring out which package has what
    priority.

    Parameters:
      arg_image (img): The preprocessed image, from which QR data is to be read
    """

    # Get the QR data from the image
    qr_result = decode(arg_image)
    changes_made = 0

    # Go through all the boxes
    if len(qr_result) is 12 and changes_made < 13:
      for box in qr_result:

        # Filter out boxes according to the rows
        # then according to the colums
        # Save the result in a dict

        # First row
        if box.rect.top < 400:

          # Leftmost
          if box.rect.left < 150 and self.package_pose_color['packagen00'] == 'NA':
            changes_made += 1
            self.package_pose_color['packagen00'] = box.data
            self.upload_on_gsheet('packagen00', str(box.data))

          # Rightmost
          elif box.rect.left > 450 and self.package_pose_color['packagen02'] == 'NA':
            changes_made += 1
            self.package_pose_color['packagen02'] = box.data
            self.upload_on_gsheet('packagen02', str(box.data))

          # Middle one
          elif self.package_pose_color['packagen01'] == 'NA':
            changes_made += 1
            self.package_pose_color['packagen01'] = box.data
            self.upload_on_gsheet('packagen01', str(box.data))

        # Second Row
        if box.rect.top > 400 and box.rect.top < 510:

          # Leftmost
          if box.rect.left < 150 and self.package_pose_color['packagen10'] == 'NA':
            changes_made += 1
            self.package_pose_color['packagen10'] = box.data
            self.upload_on_gsheet('packagen10', str(box.data))

          # Rightmost
          elif box.rect.left > 450 and self.package_pose_color['packagen12'] == 'NA':
            changes_made += 1
            self.package_pose_color['packagen12'] = box.data
            self.upload_on_gsheet('packagen12', str(box.data))

          # Middle one
          elif self.package_pose_color['packagen11'] == 'NA':
            changes_made += 1
            self.package_pose_color['packagen11'] = box.data
            self.upload_on_gsheet('packagen11', str(box.data))

        # Third Row
        if box.rect.top > 510 and box.rect.top < 700:

          # Leftmost
          if box.rect.left < 150 and self.package_pose_color['packagen20'] == 'NA':
            changes_made += 1
            self.package_pose_color['packagen20'] = box.data
            self.upload_on_gsheet('packagen20', str(box.data))

          # Rightmost
          elif box.rect.left > 450 and self.package_pose_color['packagen22'] == 'NA':
            changes_made += 1
            self.package_pose_color['packagen22'] = box.data
            self.upload_on_gsheet('packagen22', str(box.data))

          # Middle one
          elif self.package_pose_color['packagen21'] == 'NA':
            changes_made += 1
            self.package_pose_color['packagen21'] = box.data
            self.upload_on_gsheet('packagen21', str(box.data))

        # Fourth Row
        if box.rect.top > 700:

          # Leftmost
          if box.rect.left < 150 and self.package_pose_color['packagen30'] == 'NA':
            changes_made += 1
            self.package_pose_color['packagen30'] = box.data
            self.upload_on_gsheet('packagen30', str(box.data))

          # Rightmost
          elif box.rect.left > 450 and self.package_pose_color['packagen32'] == 'NA':
            changes_made += 1
            self.package_pose_color['packagen32'] = box.data
            self.upload_on_gsheet('packagen32', str(box.data))

          # Middle one
          elif self.package_pose_color['packagen31'] == 'NA':
            changes_made += 1
            self.package_pose_color['packagen31'] = box.data
            self.upload_on_gsheet('packagen31', str(box.data))

    # Check if all the package info is collected
    if len(self.package_pose_color) == 12 and changes_made == 12:
      rospy.logwarn(">>> All Package Info Acquired")
      self.all_info_aquired = True

      # Iteratre throught the packages to publish the info
      for package in self.package_pose_color:
        info = package + "_" + self.package_pose_color[package]
        rospy.loginfo(">>> Publishing")
        rospy.loginfo(info)
        self.pub_handle.publish(info)
  

  def callback(self,data):
    """
    Callback function to "/eyrc/vb/camera_1/image_raw" ROS topic.

    Parameters:
      data (ROS messaage): Incoming message from "/eyrc/vb/camera_1/image_raw" ROS topic
    """

    try:
        # Convert the incoming to data to openCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    
    except CvBridgeError as e:
      rospy.logerr(e)

    # Extract the rows, colums and channel info from the image
    (rows,cols,channels) = cv_image.shape
    
    image = cv_image

    # Resize a 720x1280 image to 360x640 to fit it on the screen
    resized_image = cv2.resize(image, (720/2, 1280/2)) 

    # cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)

    # If info on all the package is not acquired
    if not self.all_info_aquired:

      # Preprocess the image
      thresh_im = self.pre_process_img(image)
      
      # Read the QR data from the image
      self.get_qr_data(thresh_im)
    
    cv2.waitKey(3)


  # Action Client Related Code ----------------------
  def on_transition(self, goal_handle):
      """
      Function called when there is a change in state of Action Client State Machine
      
      Parameters:
          goal_handle: goal_handle of the goal, whose state changed
      """
   
      # from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.
      result = msgRosIotResult()

      index = 0
      for i in self._goal_handles:
          if self._goal_handles[i] == goal_handle:
              index = i
              break

      rospy.loginfo("[CAMERA CLIENT] Transition Callback. Client Goal Handle #: " + str(index))
      rospy.loginfo("[CAMERA CLIENT] Comm. State: " + str(goal_handle.get_comm_state()) )
      rospy.loginfo("[CAMERA CLIENT] Goal Status: " + str(goal_handle.get_goal_status()) )
      
      # Comm State - Monitors the State Machine of the Client which is different from Server's
      # Comm State = 2 -> Active
      # Comm State = 3 -> Wating for Result
      # Comm State = 7 -> Done
      
      # if (Comm State == ACTIVE)
      if goal_handle.get_comm_state() == 2:
          rospy.loginfo("[CAMERA CLIENT] " + str(index) + ": Goal just went active.")
      
      # if (Comm State == DONE)
      if goal_handle.get_comm_state() == 7:
          rospy.loginfo("[CAMERA CLIENT] " + str(index) + ": Goal is DONE")
          rospy.loginfo("[CAMERA CLIENT] " + str(goal_handle.get_terminal_state()))
          
          # get_result() gets the result produced by the Action Server
          result = goal_handle.get_result()
          rospy.loginfo("[CAMERA CLIENT] " + str(result.flag_success))

          if (result.flag_success == True):
              rospy.loginfo("[CAMERA CLIENT] Goal successfully completed. Client Goal Handle #: " + str(index))
              self.packages_updated_on_gsheet += 1
          else:
              rospy.logerr("[CAMERA CLIENT] Goal failed. Client Goal Handle #: " + str(index))

      # if all the package info updated on gsheet
      # kill this node to free up resources
      if self.packages_updated_on_gsheet >=12:
          rospy.logwarn("[CAMERA CLIENT] >>> All Package info published")
          rospy.logwarn("[CAMERA CLIENT] *** Shutting down this node")
          rospy.signal_shutdown("All info published")

  def send_goal(self, arg_protocol, arg_mode, arg_topic, arg_message):
      """
      Function used to send goals to the action server

      NOTE -
      We have not changed the contents of the action msg,
      as we were not sure if that is allowed or not.
      Hence, parameters name and it's function will not make
      much sense, since we are using it for diffrent purposes
      than its name      
      
      Parameters:
          arg_protocol (str): goal_protocol to be used
                        google_apps: To send data to google sheet
                        mqtt: To send data to mqtt server
          arg_mode (str): Mode of the goal (now depricated i.e. value should be "NA")
          arg_topic (str): Name of the sheet on which to upload data
          arg_message (str): Contents of the data to be uploaded
      
      Returns:
          goal_handle: goal_handle of the sent goal
      """

      # Create a Goal Message Object
      goal = msgRosIotGoal()

      goal.protocol = arg_protocol
      goal.mode = arg_mode
      goal.topic = arg_topic
      goal.message = arg_message

      # Send the goal
      goal_handle = self._ac.send_goal(goal, self.on_transition, None)
      rospy.loginfo("[CAMERA CLIENT] Sending the goal")

      return goal_handle

  def upload_on_gsheet(self, package, color):
    """
    Function used to upload the data to google sheet, by sending goal to Action Server

    Parameters:
        package (str): Name of the package, whose data is being sent
        color (str): Color of the package whose data is being sent
    """

    # Assign the goal number based on the package name
    goal_no = package[-2:]

    # Give the message in the form of "packagen22_green"
    message = package + "_" + color

    # Send the goal
    goal_handle = self.send_goal("google_apps", "NA", "Inventory", message)
    self._goal_handles[goal_no] = goal_handle
    rospy.loginfo("[CAMERA CLIENT] {} Goal sent".format(goal_no))


def main(args):
  """
  The main function, execution starts from here
  """
  
  # Initializing the node
  # disable_signals parameter is given to kill the node, when all the info is published
  # this is done inorder to free up the resources
  rospy.init_node('node_package_info', anonymous=True, disable_signals=True)

  # Give other nodes time to startup
  rospy.logwarn("[CAMERA CLIENT] Waiting 20 Seconds to let other nodes startup")
  rospy.sleep(20)

  # Initialize the class
  ic = CameraActionClient()

  # Not letting the node die
  try:
    rospy.spin()

  except KeyboardInterrupt:
    rospy.loginfo("Shutting down due to {}".format("KeyboardInterrupt"))
  
  # Once the program is completed, close all openCV windows
  cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
      main(sys.argv)
    except rospy.ROSInterruptException:
        pass
