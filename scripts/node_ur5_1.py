#! /usr/bin/env python

"""
"node_ur5_1" is used to control everything related to UR5_1 Arm.
Not only controlling the arm but also, which package to approach etc.
"""

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import threading

import yaml
import sys

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg

from std_msgs.msg import String

from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal
from pkg_ros_iot_bridge.msg import msgRosIotResult

from pkg_ros_iot_bridge.msg import msgMqttSub
from pkg_task5.msg import onConveyor

from datetime import datetime


class Ur5MoveitActionClient:
    """
    This class is used for controlling UR5_1 Arm.
    """

    def __init__(self):
        """
        The constructor for Ur5MoveitActionClient.

        This function will initalize this node as ROS Action Client.
        Moveit configurations for UR5_1 will also be setup here
        """

        # Configuing Moveit
        self._robot_ns = '/'  + 'ur5_1'
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

        # Make it a publisher to publish on '/eyrc/vb/onConveyor'
        self.onConveyor_pub_handle = rospy.Publisher("/eyrc/vb/onConveyor", onConveyor, queue_size = 10)

        # A dict to store all the processed info on color of pkgs
        self.package_pose_color = {}

        # A dict to store all the incoming orders
        self.orders_to_be_completed = {}

        # Lists to store order number of their respective priority
        # this helps to deliver higher priority order first
        self.orders_to_be_completed_RED = []
        self.orders_to_be_completed_YELLOW = []
        self.orders_to_be_completed_GREEN = []

        self.currently_processing = False

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        # Setup to play all the stored trajectories
        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )

        # Making it an Action Client
        self._ac = actionlib.ActionClient('/action_ros_iot',
                                            msgRosIotAction)

        # Dictionary to store the goal handles
        self._goal_handles = {}

        # Wait for the action server to start
        rospy.loginfo("[UR5_1] Waiting for Action Server ...")
        self._ac.wait_for_server()

        rospy.loginfo('\033[94m' + "[UR5_1]: >>> Ur5Moveit init done." + '\033[0m')


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

        # Form a complete file including it's name
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
                rospy.logwarn("[UR5_1]: Last Attempt to reach the pose: {}".format(arg_file_name))
                rospy.logwarn("[UR5_1]: Last Attempt Success: {}".format(flag_success))
        
        # return True

    def activate_gripper(self, activate_vacuum_gripper):
        """
        Function called to activate the vaccum gripper

        Parameters:
            activate_vaccum_gripper (bool): True, if to be activated
                                            False otherwise
        """
        in_use = True

        # Check if the gripper is being used by UR5_2 arm
        while in_use:

            param_in_use = rospy.get_param('vacuum_gripper_plugin_in_usage')
            if not param_in_use:
                in_use = False
                break
    
            elif param_in_use:

                # If in use wait for 0.1 seconds
                in_use = True
                rospy.logwarn("[UR5_1] Vaccum gripper in use, waiting...")
                rospy.sleep(0.1)

        # Call the Gripper service
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', 
                                    vacuumGripper)
        try:
            result = gripper(activate_vacuum_gripper)

        except rospy.ServiceException as exc:
            rospy.loginfo("[UR5_1]: *** Gripper Error: " + str(exc))
 
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
            rospy.logerr("[UR5_1]: *** Conveyor Error: " + str(exc))
            rospy.sleep(1)

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
            if package[-2] != '3':
                self.package_pose_color.update({package:color})


    # Action Client related code --------------------
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

        rospy.loginfo("[UR5_1] Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("[UR5_1] Comm. State: " + str(goal_handle.get_comm_state()) )
        rospy.loginfo("[UR5_1] Goal Status: " + str(goal_handle.get_goal_status()) )

        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done

        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
          rospy.loginfo("[UR5_1] " + str(index) + ": Goal just went active.")
        
        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
          rospy.loginfo("[UR5_1] " + str(index) + ": Goal is DONE")
          rospy.loginfo("[UR5_1] " + str(goal_handle.get_terminal_state()))
          
          # get_result() gets the result produced by the Action Server
          result = goal_handle.get_result()
          rospy.loginfo("[UR5_1] " + str(result.flag_success))

          if (result.flag_success == True):
            rospy.loginfo("[UR5_1] Goal successfully completed. Client Goal Handle #: " + str(index))
          
          else:
              rospy.logerr("[UR5_1] Goal failed. Client Goal Handle #: " + str(index))

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
        rospy.loginfo("[UR5_1] Sending the goal")

        # Send the goal
        goal_handle = self._ac.send_goal(goal, self.on_transition, None)

        return goal_handle

    def upload_on_gsheet(self, package, message):
        """
        Function used to upload the data to google sheet, by sending goal to Action Server

        Parameters:
            package (str): Name of the package, whose data is being sent
            message (str): Content to be uploaded on google sheet
        """

        # Assign a goal number based on package name
        goal_no = package[-2:]
        rospy.logwarn(message)

        # Send the goal
        goal_handle = self.send_goal("google_apps", "NA", "OrdersDispatched", message)
        self._goal_handles[goal_no] = goal_handle
        rospy.loginfo("[UR5_1] Goal sent")

    def save_order_to_dict(self, msg):
        """
        Function to save the incoming orders to a dictionary

        Parameters:
            msg (str): Incoming message (order) contents
        """
        # The received data is of the json format
        # we are not using the json library, so
        # doing string operations to extract the data

        # Remove the curly braces
        msg = msg[1:-1]

        # Remove the inverted commas
        msg = msg.replace('"', '')

        # Remove the extra spaces
        msg = msg.replace(" ","")

        # Split it among it's constituents
        msg = msg.split(",")

        # Extract the order ID
        order_info = msg[2].split(":")
        order_id = order_info[1]

        # Now we will make another dictionary within
        # the original dictionary, this new dictionary
        # will have order_id as it's key

        self.orders_to_be_completed[order_id] = {}

        # Storing the data
        for data in msg:
            data = data.split(":")
            self.orders_to_be_completed[order_id][data[0]] = data[1]        

        rospy.loginfo("Active Orders: {}".format(self.orders_to_be_completed))

        # Storing the orders as per their priority, in their respective list
        if self.orders_to_be_completed[order_id]["item"] == "Medicine":
            self.orders_to_be_completed_RED.append(order_id)
            rospy.loginfo("Active High Priority Orders {}".format(self.orders_to_be_completed_RED))

        elif self.orders_to_be_completed[order_id]["item"] == "Food":
            self.orders_to_be_completed_YELLOW.append(order_id)
            rospy.loginfo("Active Medium Priority Orders {}".format(self.orders_to_be_completed_YELLOW))
        
        elif self.orders_to_be_completed[order_id]["item"] == "Clothes":
            self.orders_to_be_completed_GREEN.append(order_id)
            rospy.loginfo("Active Low Priority Orders {}".format(self.orders_to_be_completed_GREEN))



    # To find the req package pose from the inventory
    def find_pkg_from_inventory(self, req_color):
        """
        Function to find the pose of the required package from the kiva pod

        Parameters:
            req_color (str): Color of the pacakge required

        Returns:
            to_traj (str): Name of the saved trajectory to reach the package
            from_traj (str): Name of the saved trajectory to reach the conveyor from package location
            pacakge_name (str): Name of the package used to fullfill the order (eg: packagen02)
        """

        # Basic flow of the function
        # Find it from the inventory
        # Delete it from the inventory
        # Return the name of the saved trajectory to play
        
        # Finding the required pacakge from the inventory
        packages = list(self.package_pose_color.keys())
        colors = list(self.package_pose_color.values())

        found = False

        try:
            package_index = colors.index(req_color)
            package_name = packages[package_index]

        except:
            # If the package is not found
            rospy.logerr("[UR5_1] Out of stock!")
            rospy.logerr("[UR5_1] Required package: {}".format(req_color))
            rospy.logerr("[UR5_1] Current Inventory: {}".format(self.package_pose_color))
            return None, None, None

        # Returning the name of the saved trajectory
        # for the package name and the name of the package
        to_traj = "c_" + package_name[-2:]
        from_traj = package_name[-2:] + "_c"

        rospy.loginfo("[UR5_1] Using {} to deliver the current order".format(package_name))
        
        # Removing the package from the dictionay of available packages
        self.package_pose_color.pop(package_name)
        rospy.loginfo("[UR5_1] Updated Inventory is as follows: \n {}".format(self.package_pose_color))

        return to_traj, from_traj, package_name

    def got_orders(self, msg):
        """
        Callback function for "/ros_iot_bridge/mqtt/sub" topic

        Parameters:
            msg (ROS message): Message object received from the ROS topic
        """

        # Convert to string data type
        msg = str(msg.message)

        # Save the order to a dictionary
        self.save_order_to_dict(msg)

    def process_orders(self):
        """
        Function used to process the order
        """

        # While orders are yet to be delivered, go through each order
        while len(self.orders_to_be_completed)>0 and not self.currently_processing:
            self.currently_processing = True
            rospy.logwarn("[UR5_1 Test] Current Orders")
            rospy.logwarn(self.orders_to_be_completed)
            rospy.logwarn(self.orders_to_be_completed_RED)
            rospy.logwarn(self.orders_to_be_completed_YELLOW)
            rospy.logwarn(self.orders_to_be_completed_GREEN)
            
            # If HP packages are to be delivered, deliver them first
            if len(self.orders_to_be_completed_RED) > 0:
                rospy.logwarn('Going for RED')
                for curr_order in self.orders_to_be_completed_RED:
                    order_id = curr_order
                    rospy.loginfo("[UR5_1] Processing for Order ID: {}".format(self.orders_to_be_completed[order_id]["order_id"]))
                    req_color = "red"
                    self.orders_to_be_completed_RED.remove(curr_order)
                    break

            # Else if MP pacakges are to be delivered, deliver them
            elif len(self.orders_to_be_completed_YELLOW) > 0:
                rospy.logwarn('Going for YELLOW')
                for curr_order in self.orders_to_be_completed_YELLOW:
                    order_id = curr_order
                    rospy.loginfo("[UR5_1] Processing for Order ID: {}".format(self.orders_to_be_completed[order_id]["order_id"]))
                    req_color = "yellow"
                    self.orders_to_be_completed_YELLOW.remove(curr_order)
                    break

            # Else if LP pacakges are to be delivered, deliver them
            elif len(self.orders_to_be_completed_GREEN) > 0:
                rospy.logwarn('Going for GREEN')
                for curr_order in self.orders_to_be_completed_GREEN:
                    order_id = curr_order
                    rospy.loginfo("[UR5_1] Processing for Order ID: {}".format(self.orders_to_be_completed[order_id]["order_id"]))
                    req_color = "green"
                    self.orders_to_be_completed_GREEN.remove(curr_order)
                    break

            # Get the to and from trajectories req to fullfill the current order along with package name
            to_traj, from_traj, package_name = self.find_pkg_from_inventory(req_color)

            # If no such package exist, skip the order
            if to_traj is None or from_traj is None:
                rospy.logwarn("[UR5_1] Skipping current order, due to above reasons")
                self.currently_processing = False
                break

            # Approach the package
            rospy.loginfo("[UR5_1] Approaching: {}".format(package_name))
            self.moveit_hard_play_planned_path_from_file(self._file_path, to_traj + '.yaml', 5)
            
            # Activate the gripper
            self.activate_gripper(True)

            # Approach the conveyor
            self.moveit_hard_play_planned_path_from_file(self._file_path, from_traj + '.yaml', 5)
            
            # Deactivate the gripper
            self.activate_gripper(False)
            rospy.loginfo("[UR5_1] Package {} on conveyor".format(package_name))
                
            # Create an object for onConveyor ROS message
            obj_onConveyor = onConveyor()
            obj_onConveyor.order_id = self.orders_to_be_completed[order_id]['order_id'] 
            obj_onConveyor.city = self.orders_to_be_completed[order_id]['city']
            obj_onConveyor.package_name = package_name

            # Publish the details of the package currently on the conveyor belt
            self.onConveyor_pub_handle.publish(obj_onConveyor)

            # Form the message, to used inorder to publish data on google sheets
            message = ""
            message = self.orders_to_be_completed[order_id]['order_id'] + ","
            message = message + self.orders_to_be_completed[order_id]['city'] + ","
            message = message + self.orders_to_be_completed[order_id]['item'] + ","
            message = message + "1,"
            message = message + "YES,"
            current_time = datetime.now()
            req_format_time = current_time.strftime("%m/%d/%Y %H:%M:%S")
            message = message + str(req_format_time)

            # Finally Sending it upload on gsheet
            self.upload_on_gsheet(package_name, message)

            rospy.logwarn("[UR5_1] Order ID {} Dispatched".format(self.orders_to_be_completed[order_id]['order_id']))
            
            # Delete it from the orders to be completed
            del self.orders_to_be_completed[order_id]

            self.currently_processing = False

    # Destructor
    def __del__(self):
        """
        Destructor for Ur5MoveitActionClient
        """
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "[UR5_1]: Object of class Ur5Moveit Deleted." + '\033[0m')


def main():
    """
    The main function, execution starts from here
    """

    rospy.sleep(2)

    # Initialize the node
    rospy.init_node('node_ur5_1', anonymous=True)

    # Initialize the class
    ur5 = Ur5MoveitActionClient()

    # Make the arm go near the conveyor
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'allZeros_c.yaml', 5)

    # Subscribe to "/pkg_task5/package_info" topic
    # to get the package info from "node_get_package_info"
    rospy.Subscriber("/pkg_task5/package_info", String, ur5.package_info)

    # Subscribe to "/ros_iot_bridge/mqtt/sub" to get the orders
    # that are comming over from MQTT
    rospy.Subscriber("/ros_iot_bridge/mqtt/sub", msgMqttSub, ur5.got_orders)

    package_info_received = False

    rospy.loginfo("[UR5_1]: Waiting for package info ...")

    # While all the package info is not received
    while not package_info_received:
        if len(ur5.package_pose_color) == 9:
            package_info_received = True
            rospy.loginfo("[UR5_1]: All Package info Received")
            print(ur5.package_pose_color)

    # Activate the conveyor at the power of 75
    ur5.activate_conveyor(75)

    # While the ROS node is not shut, process the orders
    while not rospy.is_shutdown():
        ur5.process_orders()

    del ur5


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass   