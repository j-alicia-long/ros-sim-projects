#!/usr/bin/env python

import rospy
import time

from geometry_msgs.msg import PoseStamped
from altitude.msg import AltitudeStamped
from moving_average import MovingAverage


class GPSAltitudeMovingAverage():

    # Node initialization
    def __init__(self):
        # Read parameter
        window_size = rospy.get_param("/gps_altitude_ma/gps_window_size", 3)
        self.ma = MovingAverage(window_size)

        # Create the publisher and subscriber
        self.pub = rospy.Publisher('/uav/sensors/gps_altitude_ma',
                                   AltitudeStamped,
                                   queue_size=1)
        self.sub = rospy.Subscriber('/uav/sensors/gps',
                                    PoseStamped, self.process_altitude,
                                    queue_size = 1)
        rospy.spin()


    def process_altitude(self, msg):
        moving_avg = AltitudeStamped()

        # add new value to the moving average list
        self.ma.add(msg.pose.position.z)

        # use MovingAverage to calculate the new moving average
        moving_avg.value = self.ma.get_average()

        # use the same timestamp as in msg ??
        moving_avg.stamp = msg.header.stamp

        # publish
        self.pub.publish(moving_avg)


if __name__ == '__main__':
    rospy.init_node('gps_altitude_ma_node')
    try:
        GPSAltitudeMovingAverage()
    except rospy.ROSInterruptException:
        pass