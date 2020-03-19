#!/usr/bin/env python
import rospy
import pyproj
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from rosflight_msgs.msg import GNSS


class gnss_ublox_m7:
    def __init__(self):
        self.ecef = pyproj.Proj(proj="geocent", ellps="WGS84", datum="WGS84")
        self.lla = pyproj.Proj(proj="latlong", ellps="WGS84", datum="WGS84")
        self.position = None
        self.velocity = None

        rospy.Subscriber("ublox_gps/fix", NavSatFix, self.fix_callback)
        rospy.Subscriber(
            "ublox_gps/fix_velocity", TwistWithCovarianceStamped, self.fix_velocity_callback
        )

        # TODO change queue size?
        self.pub = rospy.Publisher("ublox_gnss_test", GNSS, queue_size=100)
	rospy.spin()

    def lla_to_ecef(self, coord_lla):
        lon, lat, alt = coord_lla
        x, y, z = pyproj.transform(self.lla, self.ecef, lon, lat, alt, radians=False)  # m
        return [x, y, z]

    def fix_callback(self, msg):
        pos_lla = [msg.latitude, msg.longitude, msg.altitude]  # lat, long, alt
        self.pos = self.lla_to_ecef(pos_lla)

        # covariance matrix is diagonal, and lat/long covariance are equal
        self.horizontal_accuracy = msg.position_covariance[0]
        self.vertical_accuracy = msg.position_covariance[8]

        if self.velocity == None: # Wait for both pos and vel to be updated
            return

        rf_msg = GNSS()
        rf_msg.header.stamp = msg.header.stamp
        rf_msg.position = self.pos
        rf_msg.velocity = self.velocity
        rf_msg.horizontal_accuracy = self.horizontal_accuracy
        rf_msg.vertical_accuracy = self.vertical_accuracy
        rf_msg.speed_accuracy = self.speed_accuracy

        self.pub.publish(rf_msg)

    def fix_velocity_callback(self, msg):
        self.velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]

        # All speed covariances are similar, so first one is picked
        self.speed_accuracy = msg.twist.covariance[0]

        if self.position == None: # Wait for both pos and vel to be updated
            return

        rf_msg = GNSS()
        rf_msg.header.stamp = msg.header.stamp
        rf_msg.position = self.pos
        rf_msg.velocity = self.velocity
        rf_msg.horizontal_accuracy = self.horizontal_accuracy
        rf_msg.vertical_accuracy = self.vertical_accuracy
        rf_msg.speed_accuracy = self.speed_accuracy

        self.pub.publish(rf_msg)
        return


if __name__ == "__main__":
    rospy.init_node("gnss_ublox_m7", anonymous=False)  # TODO change to true?
    try:
        gnss_ublox_m7()
    except:
        rospy.ROSInterruptException
