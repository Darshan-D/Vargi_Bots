#!/usr/bin/env python

"""
"node_action_server" is used to get the data from MQTT server, 
as well as it is used to publish the data on the google sheets.

Basically, it handles everything outside the ROS ecosystem

"""

import rospy
import actionlib
import threading
import requests
import json

from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal
from pkg_ros_iot_bridge.msg import msgRosIotResult
from pkg_ros_iot_bridge.msg import msgRosIotFeedback

from pkg_ros_iot_bridge.msg import msgMqttSub

from pyiot import iot
from datetime import datetime


class RosIotBridgeActionServer:
	"""
	This class used as an ROS action server to handle IoT related stuff.
	"""
	
	def __init__(self):
		"""
		The constructor for the RosIotBridgeActionServer class.

		This function will initialize this node as an action server. We
		will also load various paramter values from the ROS parameter 
		server, and start a seprate thread to read the data coming from
		MQTT client 
		"""

		# Initialize the action server
		self._as = actionlib.ActionServer('/action_ros_iot',
											msgRosIotAction,
											self.on_goal,
											auto_start = False)
		# * self.on_goal - Pointer of the function to be called 
		# when a goal is received

		# * self.on_cancel - Pointer of the function to be called
		# when a cancel req is received


		# Read and Store the IOT Configuration from parameter server
		param_config_iot = rospy.get_param('config_pyiot')

		# Loading the MQTT Parameters
		self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
		self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
		self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
		self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
		self._config_mqtt_qos = param_config_iot['mqtt']['qos']
		self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']

		# Loading the Google Sheet Parameter
		self._config_sheet_id = param_config_iot['google_apps']['spread_sheet_id']
		self._config_sheet_url = param_config_iot['google_apps']['spread_sheet_url']
		self._config_email_id = param_config_iot['google_apps']['email_id']
		rospy.logwarn("email_id {}".format(self._config_email_id))
		
		# Overwriting for Eyantra's Gsheet
		# self._config_sheet_url = "https://script.google.com/macros/s/AKfycbw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7/exec"

		# Loading the dictionary for each sheets
		self.dict_IncomingOrders = {"id": "IncomingOrders",
							   "Team Id": "VB#1083",
							   "Unique Id": "iOeCqZLI",
							   "Order ID": "NA",
							   "Order Date and Time": "NA",
							   "Item": "NA",
							   "Priority": "NA",
							   "Order Quantity": "NA",
							   "City": "NA",
							   "Longitude": "NA",
							   "Latitude": "NA",
							   "Cost": "NA"}

		self.dict_Inventory = {"id": "Inventory",
								"Team Id": "VB#1083",
								"Unique Id": "iOeCqZLI",
								"SKU": "NA",
								"Item": "NA",
								"Priority": "NA",
								"Storage Number": "NA",
								"Cost": "NA",
								"Quantity": "NA",
								}

		self.dict_OrdersDispatched = {"id": "OrdersDispatched",
									"Team Id": "VB#1083",
									"Unique Id": "iOeCqZLI",
									"Order ID":"NA",
									"City":"NA",
									"Item":"NA",
									"Priority":"NA",
									"Dispatch Quantity":"NA",
									"Cost":"NA",
									"Dispatch Status":"NA",
									"Dispatch Date and Time":"NA",
									"email_id": self._config_email_id}

		self.dict_OrdersShipped = {"id":"OrdersShipped",
									"Team Id":"VB#1083",
									"Unique Id":"iOeCqZLI",
									"Order ID":"NA",
									"City":"NA",
									"Item":"NA",
									"Priority":"NA",
									"Shipped Quantity":"NA",
									"Cost":"NA",
									"Shipped Status":"NA",
									"Shipped Date and Time":"NA",
									"Estimated Time of Delivery":"NA",
									"email_id": self._config_email_id}
									

		# Initialize the ros topic '/ros_iot_bridge/mqtt/sub' so that
		# other ROS nodes can listen to MQTT messages
		self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic,
												msgMqttSub,
												queue_size = 10)

		# Subscribe to MQTT topic 'eyrc/iOeCqZLl/iot_to_ros' so that
		# it can later on publish the message to '/ros_iot_bridge/mqtt/sub'
		# for other ROS Nodes to listen
		ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
												self._config_mqtt_server_url,
												self._config_mqtt_server_port,
												self._config_mqtt_sub_topic,
												self._config_mqtt_qos)

		# * mqtt_sub_callback - Function which will be called when this node
		# receives the message from MQTT
		# * other arguments - probably required to make connection to
		# appropiate server (assumption)

		if (ret == 0):
			rospy.loginfo("[BRIDGE] MQTT Subscribe Thread Started")

		else:
			rospy.logerr("[BRIDGE] Failed to start MQTT Subscribe Thread")

		# Start the Action Server
		self._as.start()

		rospy.loginfo("[BRIDGE] Started ROS IOT Bridge")


	def mqtt_sub_callback(self, client, userdata, message):
	#def mqtt_sub_callback(self, message):
		"""
		The callback function for when a message is received from MQTT

		This function publishes the message from MQTT to a ROS topic,
		and uploads it to google sheet.

		Parameters:
			client: Client from where we are getting MQTT data
			userdata: Data of the client
			message: Incomming message from MQTT
		"""

		# Decode the message using UTF-8 and convert it
		# to 'string' datatype
		payload = str(message.payload.decode("utf-8"))

		rospy.loginfo("[BRIDGE] Message Received from MQTT")

		# Give the appropiate values to the contents of the message
		# that will be published to '/ros_iot_bridge/mqtt/sub'
		msg_mqtt_sub = msgMqttSub()
		msg_mqtt_sub.timestamp = rospy.Time.now()
		msg_mqtt_sub.topic = message.topic
		msg_mqtt_sub.message = payload

		# Publish the message
		self._handle_ros_pub.publish(msg_mqtt_sub)

		# Upload to Google Sheet
		ret = self.update_gsheet("None", True, payload)


	def on_goal(self, goal_handle):
		"""
		This function will be called when the Action Server receives a goal.

		Parameters:
			goal_handle: The goal handle of the current goal.
		"""

		# Get the goal corresponding to the current goal handle
		goal = goal_handle.get_goal()

		rospy.loginfo("[BRIDGE] Received a goal from client")

		# Validate goal parameter before publishing data to MQTT
		if(goal.protocol == 'google_apps'):

			# Set the goal as accepted
			goal_handle.set_accepted()
			rospy.logwarn("State Accepted")

			# Start processing the goal
			self.process_goal(goal_handle)

		else:

			# Set the Goal as rejected
			goal_handle.set_rejected()
			rospy.logwarn("State Rejected")
			return
	
	def process_goal(self, goal_handle):
		"""
		Function called to process the goal, after it's parameters are verified.
		
		Parameters:
			goal_handle: The goal handle of the goal to be processed.
		"""

		flag_success = False

		# Initialize the object for msgRosIotResult
		result = msgRosIotResult()

		# Get the ID of the current goal handle
		goal_id = goal_handle.get_goal_id()
		rospy.loginfo("[BRIDGE] Processing Goal: " + str(goal_id.id))
		rospy.logwarn("Processing Goal")

		# Get the current goal using goal handle
		goal = goal_handle.get_goal()

		# Upload the data to google sheet
		ret = self.update_gsheet(goal, False, None)

		if (ret == "success"):
			# If uploaded successfully
			result.flag_success = True

			# Set the goal status as succeeded
			goal_handle.set_succeeded(result)
			rospy.logwarn("Goal Suceeded")

		else:
			# If not uploaded successfully
			rospy.logerr("[BRIDGE] Failed at a Goal, ID: {}".format(goal_handle))
			result.flag_success = False

			# Set state as aborted / failed
			goal_handle.set_aborted(result)

		rospy.loginfo("[BRIDGE] Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

	def on_cancel(self, goal_handle):
		"""
		Function is called when a cancel request is received.

		Parameters:
			goal_handle: Goal handle of the goal to be cancelled.
		"""
		rospy.loginfo("[BRIDGE] Received cancel request.")
		goal_id = goal_handle.get_goal_id()


	def update_gsheet(self, goal, calling_from_within, payload):
		"""
		Function to upload the data on google sheet

		Parameters:
			goal: The goal whose data is to be uplaoded on google sheet.
			calling_from_within (bool): True if called from the action server itself.
										False otherwise.
			payload (str): The content to be uploaded on google sheet.

		Returns:
			response.content (bool): True if data uploaded successfully.
									 False otherwise.
		"""

		# If called from within the action server
		if calling_from_within:

			# Read the JSON data
			loaded_json = json.loads(payload)

			# Save the order attributes as a dictionary
			self.dict_IncomingOrders['City'] = loaded_json['city']
			rospy.logwarn("TIME {}".format(loaded_json['order_time']))
			self.dict_IncomingOrders['Order Date and Time'] = loaded_json['order_time']
			self.dict_IncomingOrders['Order ID'] = loaded_json['order_id']
			self.dict_IncomingOrders['Order Quantity'] = loaded_json['qty']
			self.dict_IncomingOrders['Item'] = loaded_json['item']
			self.dict_IncomingOrders['Latitude'] = loaded_json['lat']
			self.dict_IncomingOrders['Longitude'] = loaded_json['lon']
			
			if loaded_json['item'] == "Clothes":
				self.dict_IncomingOrders['Priority'] = "LP"
				self.dict_IncomingOrders['Cost'] = "50"

			if loaded_json['item'] == "Food":
				self.dict_IncomingOrders['Priority'] = "MP"
				self.dict_IncomingOrders['Cost'] = "100"

			if loaded_json['item'] == "Medicine":
				self.dict_IncomingOrders['Priority'] = "HP"
				self.dict_IncomingOrders['Cost'] = "150"

			# Upload the data to google sheet
			response = requests.get(self._config_sheet_url, params = self.dict_IncomingOrders)
			rospy.loginfo("[BRIDGE] Gsheet Update Status: {}".format(response.content))
			return response.content

		# If not called from within the action server
		else:

			# If data is to be uploaded in Inventory sheet
			if goal.topic == "Inventory":

				# Split the message, and store the contents of the 
				# message as a key value pair in a dictionary

				# All the attributes are seprated by a underscore, so split it 
				package_info = goal.message.split("_")
				package_name = package_info[0]
				package_color = package_info[1]
				
				storage_number = "R"+package_name[-2] + " "+"C"+package_name[-1]
				self.dict_Inventory['Storage Number'] = storage_number
				self.dict_Inventory['Quantity'] = "1"
				
				# For SKU
				today = datetime.today()
				today = str(today)
				today = today.split(" ")
				date = today[0].split("-")
				
				year = date[0][-2:]
				month = date[1]

				inventory_pose = package_name[-2] + package_name[-1]

				# Figure out diffrent attributes from the package color
				if package_color == "red":
					self.dict_Inventory['Item'] = "Medicine"
					self.dict_Inventory['Priority'] = "HP"
					self.dict_Inventory['Cost'] = "150"
					self.dict_Inventory['SKU'] = "R"+inventory_pose+month+year

				elif package_color == "yellow":
					self.dict_Inventory['Item'] = "Food"
					self.dict_Inventory['Priority'] = "MP"
					self.dict_Inventory['Cost'] = "100"
					self.dict_Inventory['SKU'] = "Y"+inventory_pose+month+year

				elif package_color == "green":
					self.dict_Inventory['Item'] = "Clothes"
					self.dict_Inventory['Priority'] = "LP"
					self.dict_Inventory['Cost'] = "50"
					self.dict_Inventory['SKU'] = "G"+inventory_pose+month+year

				gsheet_paramters = self.dict_Inventory


			# If data is to be uploaded in OrdersDispatched sheet
			elif goal.topic == "OrdersDispatched":

				# Split the message, and store the contents of the 
				# message as a key value pair in a dictionary

				# All the attributes are seprated by a comma, so spliting it 
				package_info = goal.message.split(",")
				
				self.dict_OrdersDispatched['Order ID'] = package_info[0]
				self.dict_OrdersDispatched['City'] = package_info[1]
				self.dict_OrdersDispatched['Item'] = package_info[2]

				# Figuring out other attributes from the type of content
				if self.dict_OrdersDispatched['Item'] == "Medicine":
					self.dict_OrdersDispatched['Priority'] = "HP"
					self.dict_OrdersDispatched['Cost'] = "150"
				
				if self.dict_OrdersDispatched['Item'] == "Food":
					self.dict_OrdersDispatched['Priority'] = "MP"
					self.dict_OrdersDispatched['Cost'] ="100"

				if self.dict_OrdersDispatched['Item'] == "Clothes":
					self.dict_OrdersDispatched['Priority'] = "LP"
					self.dict_OrdersDispatched['Cost'] = "50"

				self.dict_OrdersDispatched['Dispatch Quantity'] = package_info[3]
				self.dict_OrdersDispatched['Dispatch Status'] = package_info[4]
				self.dict_OrdersDispatched['Dispatch Date and Time'] = package_info[5]


			 	gsheet_paramters = self.dict_OrdersDispatched

			# If data is to be uploaded in OrdersShipped sheet
			elif goal.topic == "OrdersShipped":

				# Split the message, and store the contents of the 
				# message as a key value pair in a dictionary

				# All the attributes are seprated by a comma, so spliting it 
				package_info = goal.message.split(",")

				self.dict_OrdersShipped['Order ID'] = package_info[0]
				self.dict_OrdersShipped['City'] = package_info[1]
				
				# Figuring out other attributs from the color of the package
				if package_info[2] == 'red':
					self.dict_OrdersShipped['Item'] = "Medicine"
					self.dict_OrdersShipped['Priority'] = "HP"
					self.dict_OrdersShipped['Cost'] = "150"
				elif package_info[2] == 'yellow':
					self.dict_OrdersShipped['Item'] = "Food"
					self.dict_OrdersShipped['Priority'] = "MP"
					self.dict_OrdersShipped['Cost'] = "100"
				elif package_info[2] == 'green':
					self.dict_OrdersShipped['Item'] = "Clothes"
					self.dict_OrdersShipped['Priority'] = "LP"
					self.dict_OrdersShipped['Cost'] = "50"

				self.dict_OrdersShipped['Shipped Quantity'] = package_info[3]
				self.dict_OrdersShipped['Shipped Status'] = "YES"
				self.dict_OrdersShipped['Shipped Date and Time'] = package_info[4]
				self.dict_OrdersShipped['Estimated Time of Delivery'] = package_info[5]

				gsheet_paramters = self.dict_OrdersShipped

			# Upload the data to google sheet
			response = requests.get(self._config_sheet_url, params = gsheet_paramters)
			rospy.loginfo("[BRIDGE] Gsheet Update Status: {}".format(response.content))
			return response.content


def main():
	"""
	The main function, execution starts from here
	"""

	# Initialize the node
	rospy.init_node("node_action_server_ros_iot_bridge")

	# Create a object for RosIotBridgeActionServer class
	action_server = RosIotBridgeActionServer()

	# Not letting this node die
	rospy.spin()


if __name__ == '__main__':
	main()