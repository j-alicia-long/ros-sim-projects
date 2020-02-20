#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import random
from sensor_simulators.srv import calibrate, calibrateResponse

# Create a class which saves the altitude of the drone and esimates the pressure
class PressureSensor():

  # Node initialization
  def __init__(self):

    # Sleep to allow simulation to start
    time.sleep(1)

    # Create the publisher and subscriber
    self.pressure_pub = rospy.Publisher('/uav/sensors/pressure', Float64, queue_size=1)
    self.position_sub = rospy.Subscriber('/uav/sensors/gps', PoseStamped, self.getPosition, queue_size = 1)
    
    # Save the altitude of the drone
    self.altitude = 0

    # Used to estimate the pressure
    self.pressure = 0

    # Determines the baseline value of the perssure sensor at height 0m
    self.baseline_value = 0

    # Adds pressure calibrate service
    self.service = rospy.Service('calibrate_pressure', calibrate, self.CalibrateFunction)

    # Call the mainloop of our class
    self.mainloop()


  # Callback for the keyboard manager
  def getPosition(self, msg):
    # Save the drones alitude
    self.altitude = msg.pose.position.z


  # Saves the baseline value
  def CalibrateFunction(self, request):
    # If we want to calibrate
    if request.zero == True:
      self.baseline_value = self.pressure
    else:
      self.baseline_value = 0
    # Return the new baseline value
    return calibrateResponse(self.baseline_value)


  # The main loop of the function
  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(10)

    # Message for sending data
    pressure_msg = Float64()

    # While ROS is still running
    while not rospy.is_shutdown():

      # Publish the altitude
      self.pressure_pub.publish(pressure_msg)

      # Compute the pressure in milibars (according to https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf)
      common_term = pow(4.43077*pow(10,6)-self.altitude, 0.2553026)
      self.pressure = (-0.0000229797 * self.altitude * common_term) + (20.3635* common_term)

      # Add sensor noise
      noise = pow(10,-5) * random.uniform(-1, 1)

      # Set the pressure
      pressure_msg.data = self.pressure + noise - self.baseline_value

      # Sleep for the remainder of the loop
      rate.sleep()


if __name__ == '__main__':
  rospy.init_node('pressure_simulation_node')
  try:
    ktp = PressureSensor()
  except rospy.ROSInterruptException:
    pass