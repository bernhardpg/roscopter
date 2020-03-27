#!/usr/bin/env python
import rospy
import pyproj
import math
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from rosflight_msgs.msg import GNSS


class gnss_ublox:
    def __init__(self):
        self.ecef = pyproj.Proj(proj="geocent", ellps="WGS84", datum="WGS84")
        self.lla = pyproj.Proj(proj="latlong", ellps="WGS84", datum="WGS84")
        self.position = None
        self.velocity = None

        rospy.Subscriber("ublox_gps/fix", NavSatFix, self.fix_callback)
        rospy.Subscriber(
            "ublox_gps/fix_velocity", TwistWithCovarianceStamped, self.fix_velocity_callback
        )

        self.pub = rospy.Publisher("gnss", GNSS, queue_size=1000)
        rospy.spin()

    def lla_to_ecef(self, coord_lla):
        lon, lat, alt = coord_lla
        x, y, z = pyproj.transform(self.lla, self.ecef, lon, lat, alt, radians=False)  # m
        return [x, y, z]

    def fix_callback(self, msg):
        position_lla = [msg.latitude, msg.longitude, msg.altitude]  # lat, long, alt
        self.position = self.lla_to_ecef(position_lla)

        # covariance matrix is diagonal, and lat/long covariance are equal
        std_x_squared = msg.position_covariance[0]
        std_y_squared = msg.position_covariance[3]
        self.horizontal_accuracy = math.sqrt(std_x_squared + std_y_squared) # Use DRMS as accuracy measure
        self.vertical_accuracy = math.sqrt(msg.position_covariance[8]) # Simply use std for 1D accuracy

        if self.velocity == None: # Wait for both pos and vel to be updated
            return

        self.publish_gnss_data(msg.header.stamp)

    def fix_velocity_callback(self, msg):
        self.velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]

        # All speed covariances are similar, so first one is picked
        self.speed_accuracy = math.sqrt(msg.twist.covariance[0]) # Use std for speed accuracy

        if self.position == None: # Wait for both pos and vel to be updated
            return

        self.publish_gnss_data(msg.header.stamp)

    def publish_gnss_data(self, msg_header_stamp):
        rf_msg = GNSS()
        rf_msg.header.stamp = msg_header_stamp
        rf_msg.position = self.position
        rf_msg.velocity = self.velocity
        rf_msg.horizontal_accuracy = self.horizontal_accuracy
        rf_msg.vertical_accuracy = self.vertical_accuracy
        rf_msg.speed_accuracy = self.speed_accuracy

        self.pub.publish(rf_msg)



if __name__ == "__main__":
    rospy.init_node("gnss_ublox", anonymous=False)
    try:
        gnss_ublox()
    except:
        rospy.ROSInterruptException
