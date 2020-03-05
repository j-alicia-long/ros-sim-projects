#!/usr/bin/env python

import rospy
import time
import copy
from keyboard.msg import Key
from geometry_msgs.msg import Vector3


# Create a class which we will use to take keyboard commands and convert them to a position
class KeyboardManager():

  # On node initialization
  def __init__(self):

    # Create the publisher and subscriber
    self.position_pub = rospy.Publisher('/keyboardmanager/position', Vector3, queue_size=1)
    self.keyboard_sub = rospy.Subscriber('/keyboard/keydown', Key, self.get_key, queue_size = 1)

    # Create the position message we are going to be sending
    self.pos = Vector3()
    self.prev_pos = Vector3()

    # Start the drone a little bit off the ground
    self.pos.z = 0.5

    # Create a variable we will use to hold the key code
    self.key_code = -1

    # Give the simulation enough time to start
    time.sleep(10)

    # Call the mainloop of our class
    self.mainloop()


  # Callback for the keyboard sub
  def get_key(self, msg):
    self.key_code = msg.code

  # Converts a position to string for printing
  def goalToString(self, msg):
    pos_str = "(" + str(msg.x) 
    pos_str += ", " + str(msg.y)
    pos_str += ", " + str(msg.z) + ")"
    return pos_str


  def mainloop(self):

    # Set the rate of this loop
    rate = rospy.Rate(20)

    # While ROS is still running
    while not rospy.is_shutdown():

      # Check if any key has been pressed
      if self.key_code == 273: # Up Arrow: +Y
        print("Up key was pressed!")
        self.pos.y += 0.5
      elif self.key_code == 274: # Down Arrow: -Y
        print("Down key was pressed!")
        self.pos.y -= 0.5
      elif self.key_code == 275: # Right Arrow: +X
        print("Right key was pressed!")
        self.pos.x += 0.5
      elif self.key_code == 276: # Left Arrow: -X
        print("Left key was pressed!")
        self.pos.x -= 0.5
      elif self.key_code == 119: # W: +Z
        print("Up key was pressed!")
        self.pos.z += 0.5
      elif self.key_code == 115: # S: -Z
        print("Up key was pressed!")
        self.pos.z -= 0.5

      # Log positions typed when they change
      check_x = self.prev_pos.x != self.pos.x
      check_y = self.prev_pos.y != self.pos.y
      check_z = self.prev_pos.z != self.pos.z
      if check_x or check_y or check_z:
        self.prev_pos = copy.deepcopy(self.pos)
        rospy.loginfo(str(rospy.get_name()) + ": Keyboard: " + self.goalToString(self.pos))

      # If the user presses the enter key
      if self.key_code == 13:
        # Publish the position
        self.position_pub.publish(self.pos)
        rospy.loginfo(str(rospy.get_name()) + ": Sending Position")
      
      # Reset the code
      if self.key_code != -1:
        self.key_code = -1

      # Sleep for the remainder of the loop
      rate.sleep()


if __name__ == '__main__':
  rospy.init_node('keyboard_manager') #careful naming
  try:
    ktp = KeyboardManager()
  except rospy.ROSInterruptException:
    pass