#!/usr/bin/env python

import rospy
import time
import copy
from enum import Enum
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from simple_control.srv import toggle_cage, toggle_cageResponse


# A class to keep track of the quadrotors state
class DroneState(Enum):
  HOVERING = 1
  VERIFYING = 2
  MOVING = 3


# Create a class which takes in a position and verifies it, keeps track of the state and publishes commands to a drone
class StateAndSafety():

  # Node initialization
  def __init__(self):

    # Create the publisher and subscriber
    self.position_pub = rospy.Publisher('/uav/input/position', Vector3, queue_size=1)
    self.keyboard_sub = rospy.Subscriber('/keyboardmanager/position', Vector3, self.getKeyboardCommand, queue_size = 1)

    # TO BE COMPLETED AFTER CHECKPOINT 1
    # TODO: Add a position_sub that subscribes to the drones pose
    self.position_sub = rospy.Subscriber('/uav/sensors/gps', PoseStamped, self.getDronePosition, queue_size = 1)

    # Save the acceptance range
    self.acceptance_range = 0.5

    # Create the drones state as hovering
    self.state = DroneState.HOVERING
    rospy.loginfo(str(rospy.get_name()) + ": Current State: HOVERING")

    # Create the goal messages we are going to be sending
    self.goal_cmd = Vector3()

    # Create a point message that saves the drones current position
    self.drone_position = Point()

    # Start the drone a little bit off the ground
    self.goal_cmd.z = 0.5

    # Keeps track of whether the position was changed or not
    self.goal_changed = False

    # Get the acceptance range
    self.acceptance_range = rospy.get_param("/state_safety_node/acceptance_range", 0.5)

    # Getting the virtual cage parameters
    cage_params = rospy.get_param('/state_safety_node/virtual_cage', {'x': 5, 'y': 5, 'z': 5})
    cx, cy, cz = cage_params['x'], cage_params['y'], cage_params['z']

    # Create the virtual cage
    self.cage_on = True
    self.cage_x = [-1 * cx, cx]
    self.cage_y = [-1 * cy, cy]
    self.cage_z = [0, cz]

    # Display incoming parameters
    rospy.loginfo(str(rospy.get_name()) + ": Launching with the following parameters:")
    rospy.loginfo(str(rospy.get_name()) + ": Param: cage x - " + str(self.cage_x))
    rospy.loginfo(str(rospy.get_name()) + ": Param: cage y - " + str(self.cage_y))
    rospy.loginfo(str(rospy.get_name()) + ": Param: cage z - " + str(self.cage_z))
    rospy.loginfo(str(rospy.get_name()) + ": Param: acceptance range - " + str(self.acceptance_range))

    # Add cage_toggle service
    self.service = rospy.Service('toggle_cage', toggle_cage, self.toggleCageFunction)

    # Call the mainloop of our class
    self.mainloop()

  # Saves the baseline value
  def toggleCageFunction(self, request):
    if request.cage_on == True:
      self.cage_on = True
    else:
      self.cage_on = False
    # Return the new baseline value
    return toggle_cageResponse(True)


  # Callback for the keyboard manager
  def getKeyboardCommand(self, msg):
    # Save the keyboard command
    if self.state == DroneState.HOVERING:
      self.goal_changed = True
      self.goal_cmd = copy.deepcopy(msg)


  # TO BE COMPLETED AFTER CHECKPOINT 1
  # TODO: Add function to receive drone's position messages
  def getDronePosition(self, msg):
    self.drone_position.x = msg.pose.position.x
    self.drone_position.y = msg.pose.position.y
    self.drone_position.z = msg.pose.position.z


  # Converts a position to string for printing
  def goalToString(self, msg):
    pos_str = "(" + str(msg.x) 
    pos_str += ", " + str(msg.y)
    pos_str += ", " + str(msg.z) + ")"
    return pos_str

  # This function is called when we are in the hovering state
  def processHovering(self):
    # Print the requested goal if the position changed
    if self.goal_changed:
      rospy.loginfo(str(rospy.get_name()) + ": Requested Position: " + self.goalToString(self.goal_cmd))
      rospy.loginfo(str(rospy.get_name()) + ": Current State: VERIFYING")
      self.state = DroneState.VERIFYING
      self.goal_changed = False

  # This function is called when we are in the verifying state
  def processVerifying(self):
    # Check if the goal position is accepted
    within_cage = (
      self.goal_cmd.x > self.cage_x[0]+self.acceptance_range
      and self.goal_cmd.x < self.cage_x[1]-self.acceptance_range
      and self.goal_cmd.y > self.cage_y[0]+self.acceptance_range
      and self.goal_cmd.y < self.cage_y[1]-self.acceptance_range
      and self.goal_cmd.z > self.cage_z[0]+self.acceptance_range
      and self.goal_cmd.z < self.cage_z[1]-self.acceptance_range
    )
    if not self.cage_on or within_cage:
      rospy.loginfo(str(rospy.get_name()) + ": Accepted Position: " + self.goalToString(self.goal_cmd))
      rospy.loginfo(str(rospy.get_name()) + ": ----------------------------------")
      rospy.loginfo(str(rospy.get_name()) + ": Current State: MOVING")
      self.state = DroneState.MOVING
    else:
      rospy.loginfo(str(rospy.get_name()) + ": Rejected Position: " + self.goalToString(self.goal_cmd))
      rospy.loginfo(str(rospy.get_name()) + ": ----------------------------------")
      rospy.loginfo(str(rospy.get_name()) + ": Current State: HOVERING")
      self.state = DroneState.HOVERING
      # Reset drone state
      self.goal_cmd.x = self.drone_position.x
      self.goal_cmd.y = self.drone_position.y
      self.goal_cmd.z = self.drone_position.z

  # This function is called when we are in the moving state
  def processMoving(self):
    # Compute the distance between requested position and current position
    dx = self.goal_cmd.x - self.drone_position.x
    dy = self.goal_cmd.y - self.drone_position.y
    dz = self.goal_cmd.z - self.drone_position.z

    # Euclidean distance
    distance_to_goal = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))

    # If goal is reached transition to hovering
    if distance_to_goal < self.acceptance_range:
      self.state = DroneState.HOVERING
      rospy.loginfo(str(rospy.get_name()) + ": Complete")
      rospy.loginfo(str(rospy.get_name()) + ": ----------------------------------")
      rospy.loginfo(str(rospy.get_name()) + ": Current State: HOVERING")

  # The main loop of the function
  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(20)

    # While ROS is still running
    while not rospy.is_shutdown():
      # Publish the position
      self.position_pub.publish(self.goal_cmd)

      # Check if the drone is in a moving state
      if self.state == DroneState.MOVING:
        self.processMoving()

      # If we are hovering then accept keyboard commands
      elif self.state == DroneState.HOVERING:
        self.processHovering()

      # If we are verifying then check position
      elif self.state == DroneState.VERIFYING:
        self.processVerifying()

      # Sleep for the remainder of the loop
      rate.sleep()


if __name__ == '__main__':
  rospy.init_node('state_safety_node')
  try:
    ktp = StateAndSafety()
  except rospy.ROSInterruptException:
    pass

