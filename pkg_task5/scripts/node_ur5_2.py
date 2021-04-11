#! /usr/bin/env python

"""
"node_ur5_2" is used to control everything related to UR5_2 Arm.
"""

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import threading

import yaml
import os
import math
import sys

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg

from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.msg import Model

from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal
from pkg_ros_iot_bridge.msg import msgRosIotResult

from std_msgs.msg import String
from pkg_task5.msg import onConveyor

from datetime import datetime, date, timedelta


class Ur5MoveitActionClient:
    """
    This class is used for controlling UR5_2 Arm.
    """

    def __init__(self):
        """
        The constructor for Ur5MoveitActionClient.

        This function will initalize this node as ROS Action Client.
        Moveit configurations for UR5_2 will also be setup here
        """

        # Configuring Moveit
        self._robot_ns = '/'  + 'ur5_2'
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        # Subscribe to "/eyrc/vb/onConveyor" to get the info on dispatched packages
        rospy.Subscriber("/eyrc/vb/onConveyor", onConveyor, self.cb_onConveyor)
        
        # A dict to store all the processed info on color of pkgs
        self.package_pose_color = {}

        # A list to store all the shipped models
        self.shipped_pkgs = []

        # A dict to store all the dispatched models
        self.dispatched_pkg ={}

        # Attribute to store computed trajectory by the planner	
        self._computed_plan = ''

        # Set to True if Currently Executing a Path
        self._exec_curr_path = False

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        # Setting up the path of the saved trajectories
        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )

        # Making it Action Client
        self._ac = actionlib.ActionClient('/action_ros_iot',
                                            msgRosIotAction)

        # Dictionary to store the goal handles
        self._goal_handles = {}

        # Wait for the action server to start
        rospy.loginfo("[UR5_2] Waiting for Action Server ...")
        self._ac.wait_for_server()

        rospy.loginfo('\033[94m' + "[UR5_2]: >>> Ur5Moveit init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):
        """
        Function to move the UR5 arm to a specific joint angles

        Parameters:
            arg_list_joint_values (list): List of joint values for the UR5 Arm

        Returns:
            flag_plan (bool): True if joint angles set successfully
                              False otherwise
        """

        list_joint_values = self._group.get_current_joint_values()
        
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)
        list_joint_values = self._group.get_current_joint_values()
        pose_values = self._group.get_current_pose().pose

        if (flag_plan == True):
            list_joint_values = self._group.get_current_joint_values()
            
        return flag_plan

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        """
        This function is used to play the saved trajectories

        Parameters:
            arg_file_path (str): path of the saved trajectory
            arg_file_name (str): name of the saved trajectory

        Returns:
            ret (bool): True, if path executed successfully
                        False otherwise
        """

        # Form a complete path along with it's name
        file_path = arg_file_path + arg_file_name
        
        # Load the file
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        
        # Execute the path
        ret = self._group.execute(loaded_plan)

        return ret

    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        """
        This function is used to play the saved trajectories several times incase of failuer

        Parameters:
            arg_file_path (str): path of the saved trajectory
            arg_file_name (str): name of the saved trajectory
            arg_max_attemps (int): maximum number of times to be retried in case of failuer
        """

        number_attempts = 0
        flag_success = False

        # Keep trying if it fails for max attemps allowed
        while ((number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            if number_attempts == arg_max_attempts:
                rospy.logwarn("[UR5_2]: Last Attempt to reach the pose: {}".format(arg_file_name))
                rospy.logwarn("[UR5_2]: Last Attempt Success: {}".format(flag_success))
        
        # return True

    def logical_camera_callback(self, msg):
        """
        Callback to the "/eyrc/vb/logical_camera_2" ROS topic

        Parameters:
            msg (ROS msg): ROS message received from the topic
        """

        # Check if any packages are there
        if len(msg.models) > 0 and not self._exec_curr_path:

            # Go through the packages detected
            for model in msg.models:

                # Ignore the UR5 arm
                if model.type != "ur5" and model.type not in self.shipped_pkgs:
                    current_package = model.type
                    current_package_color = self.package_pose_color[model.type]

                    # Check the packae color, and set the respective trajectories
                    if current_package_color == "red":
                        bin_trajectory = 'c_r.yaml'
                        ret_trajectory = 'r_c.yaml'

                    if current_package_color == "yellow":
                        bin_trajectory = 'c_y.yaml'
                        ret_trajectory = 'y_c.yaml'

                    if current_package_color == "green":
                        bin_trajectory = 'c_g.yaml'
                        ret_trajectory = 'g_c.yaml'
            
                    # If the package comes near/under the end effector
                    if model.pose.position.y < 0.11 and model.pose.position.y > -0.11:

                        # Slow down the conveyor
                        self.activate_conveyor(20)
                        rospy.loginfo("[UR5_2]: Serving Package: {}".format(current_package))
                        rospy.loginfo("[UR5_2]: Will go to bin {}".format(current_package_color))
                        rospy.loginfo("---")

                        # Ship the package
                        self.activate_gripper(True)
                        self.moveit_hard_play_planned_path_from_file(self._file_path, bin_trajectory, 5)
                        self.activate_conveyor(40)
                        self.activate_gripper(False)

                        # Upload to google sheet
                        if model.type in self.dispatched_pkg:
                            print("Starting Shipping Project")

                            # Use a diffrent thread to upload to google sheet
                            # as we don't wanna wait till the upload on gsheet
                            # gets complete
                            thread_to_gsheet = threading.Thread(name = 'shipped_ghseet',
                                                                target = self.upload_on_ghseet,
                                                                args = (model.type, ))
                            thread_to_gsheet.start()

                        # Play the return trajectory
                        self.moveit_hard_play_planned_path_from_file(self._file_path, ret_trajectory, 5)
                        self.shipped_pkgs.append(model.type)


                        # Shutdown this node after all pkgs are sorted
                        if len(self.shipped_pkgs) == 9:
                            rospy.logwarn(" *** Shutting down this node")
                            rospy.signal_shutdown("All packages sorted")

    def activate_gripper(self, activate_vacuum_gripper):
        """
        Function called to activate the vaccum gripper

        Parameters:
            activate_vaccum_gripper (bool): True, if to be activated
                                            False otherwise
        """

        in_use = True

        # Check if the gripper is being used by UR5_1 arm
        while in_use:
            param_in_use = rospy.get_param('vacuum_gripper_plugin_in_usage')
            if not param_in_use:
                in_use = False
                break
    
            elif param_in_use:

                # If in use wait for 0.1 seconds
                in_use = True
                rospy.logwarn("[UR5_2] Vaccum gripper in use, waiting...")
                rospy.sleep(0.1)

        # Call the Gripper service
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', 
                                    vacuumGripper)
        try:
            result = gripper(activate_vacuum_gripper)

        except rospy.ServiceException as exc:
            rospy.loginfo("[UR5_2]: *** Gripper Error: " + str(exc))
    
    def activate_conveyor(self, power):
        """
        Function called to activate the conveyor belt.

        Parameters:
            power (int): The power at which to run the conveyor
                         11- minimum, 100- maximum
        """

        # Call the conveyor service
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        conveyor = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', 
                                    conveyorBeltPowerMsg)

        try:
            result = conveyor(power)

        except rospy.ServiceException as exc:
            rospy.logerr("[UR5_2]: *** Conveyor Error: " + str(exc))

    def package_info(self, data):
        """
        Callback to "/pkg_task5/package_info" topic used to get pkg name and color
        
        Parameters:
            data (ROS message): Message received from "/pkg_task5/package_info" topic
        """

        # Extract the data from the message
        data = str(data.data)

        # All the attributes are seprated by a underscore, so split it
        data = data.split("_")

        # Store the package pose and color as key value pair
        if len(self.package_pose_color) < 13:
            package, color = data[0], data[1]
            self.package_pose_color.update({package:color})

    def cb_onConveyor(self, msg):
        """
        Callback to "/eyrc/vb/onConveyor" ROS topic

        Parameters:
            msg (ROS msg): Message received from "/eyrc/vb/onConveyor" ROS topic
        """

        order_id = msg.order_id
        city = msg.city
        package_name = msg.package_name
        self.dispatched_pkg[package_name] = {}
        self.dispatched_pkg[package_name].update({'City':city})
        self.dispatched_pkg[package_name].update({'Order ID':order_id})
        print(self.dispatched_pkg)


    # Action Client Related Code -----
    def on_transition(self, goal_handle):
        """
        Function called when there is a change in state of Action Client State Machine
        
        Parameters:
            goal_handle: goal_handle of the goal, whose state changed
        """

        result = msgRosIotResult()

        index = 0
        for i in self._goal_handles:
          if self._goal_handles[i] == goal_handle:
              index = i
              break

        rospy.loginfo("[UR5_2] Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("[UR5_2] Comm. State: " + str(goal_handle.get_comm_state()) )
        rospy.loginfo("[UR5_2] Goal Status: " + str(goal_handle.get_goal_status()) )

        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done

        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
          rospy.loginfo("[UR5_2] " + str(index) + ": Goal just went active.")

        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
          rospy.loginfo("[UR5_2] " + str(index) + ": Goal is DONE")
          rospy.loginfo("[UR5_2] " + str(goal_handle.get_terminal_state()))
          
          # get_result() gets the result produced by the Action Server
          result = goal_handle.get_result()
          rospy.loginfo("[UR5_2] " + str(result.flag_success))

          if (result.flag_success == True):
              rospy.loginfo("[UR5_2] Goal successfully completed. Client Goal Handle #: " + str(index))
          else:
              rospy.logerr("[UR5_2] Goal failed. Client Goal Handle #: " + str(index))

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

        # Creating a Goal Message Object
        goal = msgRosIotGoal()
        goal.protocol = arg_protocol
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.message = arg_message
        rospy.loginfo("[UR5_2] Sending the goal")

        # Send the goal
        goal_handle = self._ac.send_goal(goal, self.on_transition, None)

        return goal_handle

    def upload_on_ghseet(self, package):
        """
        Function used to upload the data to google sheet, by sending goal to Action Server

        Parameters:
            package (str): Name of the package, whose data is being sent
        """

        # Assign a goal number based on package name
        goal_no = package[-2:]
        message = self.get_message_ready(package)
        rospy.logwarn(message)

        # Send the goal
        goal_handle = self.send_goal("google_apps", "NA", "OrdersShipped", message)
        self._goal_handles[goal_no] = goal_handle
        rospy.loginfo("[UR5_2] Goal Sent")

    # Function to fill the required details in the goal message
    def get_message_ready(self, package):
        """
        Function to get the get the message ready, to be uploaded on google sheet

        Parameters:
            package (str): Name of the package whose details are to be uploaded

        Returns:
            message (str): Perfectly formated message which can be uploaded to google sheets
        """

        order_id = self.dispatched_pkg[package]['Order ID'] + ','
        city = self.dispatched_pkg[package]['City'] + ','
        pkg_color = self.package_pose_color[package] + ','
        shipped_qty = '1' + ','
        current_time = datetime.now()
        req_format_time = current_time.strftime("%m/%d/%Y %H:%M:%S")
        shipped_date = req_format_time + ','

        curr_date = date.today()
        if self.package_pose_color[package] == 'red':
            delivery_in = 1
        elif self.package_pose_color[package] == 'yellow':
            delivery_in = 3
        elif self.package_pose_color[package] == 'green':
            delivery_in = 5

        estimated_delivery = curr_date + timedelta(delivery_in)
        estimated_delivery = str(estimated_delivery)

        message = order_id + city + pkg_color + shipped_qty + shipped_date + estimated_delivery

        return message

    def __del__(self):
        """
        The destructor for Ur5MoveitActionClient class
        """
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "[UR5_2]: Object of class Ur5Moveit Deleted." + '\033[0m')


def main():
    """
    The main function, execution starts from here
    """

    rospy.sleep(4)

    # Initialize the node
    # disable_signals parameter is given to kill the node, when all pkgs are sorted
    rospy.init_node('node_ur5_2', anonymous=True, disable_signals=True)

    # Initialize the class
    ur5 = Ur5MoveitActionClient()

    # Joint angles for a pose near the conveyor
    angle_conveyor = [math.radians(math.degrees(0.00244275758744)),
                        math.radians(math.degrees(-2.25268166845)),
                        math.radians(math.degrees(-1.36316049135)),
                        math.radians(math.degrees(-1.09563942938)),
                        math.radians(math.degrees(1.57039053507)),
                        math.radians(math.degrees(0.00306019660049))]

    # Set the robot to that joint angles
    ur5.set_joint_angles(angle_conveyor)

    # Subscribe to "/pkg_task5/package_info" topic
    # to get the package info
    rospy.Subscriber("/pkg_task5/package_info", String, ur5.package_info)

    package_info_received = False

    rospy.loginfo("[UR5_2]: Waiting for package info ...")

    # While all the package info is not received
    while not package_info_received:
        if len(ur5.package_pose_color) == 12:
            package_info_received = True
            rospy.loginfo("[UR5_2]: All Package info Received")  

    # Subscribing to the logical camera topic
    rospy.Subscriber("/eyrc/vb/logical_camera_2", 
                    LogicalCameraImage, ur5.logical_camera_callback)

    # Stop this node from closing prematurely
    rospy.spin()

    # Calling the Destructor
    del ur5


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass