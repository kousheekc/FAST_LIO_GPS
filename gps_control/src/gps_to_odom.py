#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

import utm
import math

class GPS2Odom:
    def __init__(self):
        rospy.Subscriber('/gps/start_pos', NavSatFix, self.gpsstartpos_cb)
        rospy.Subscriber('/gps/start_ori', Float64, self.gpsstartori_cb)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gpssensor_cb)
        rospy.Subscriber('/gps/foxglove', NavSatFix, self.gpsfoxglove_cb)

        rospy.Subscriber('/livox/imu', Imu, self.latesttime_cb)


        self.odom_pub = rospy.Publisher('/odometry/gps', Odometry, queue_size=10)

        self.previous_x = 0
        self.previous_y = 0

        self.start_lat = None
        self.start_lon = None
        self.start_ori = 0
        self.latest_stamp = 0

    def latesttime_cb(self, data: Imu):
        self.latest_stamp = data.header.stamp

    def dist(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def gps2odom(self, lat: float, lon: float):
        currentUTM = utm.from_latlon(lat, lon);
        startUTM = utm.from_latlon(self.start_lat, self.start_lon);

        eastDelta = currentUTM[0] - startUTM[0];
        northDelta = currentUTM[1] - startUTM[1];

        x = math.cos(math.radians(self.start_ori)) * eastDelta + math.sin(math.radians(self.start_ori)) * northDelta;
        y = -math.sin(math.radians(self.start_ori)) * eastDelta + math.cos(math.radians(self.start_ori)) * northDelta;

        return x, y;

    def gpsstartpos_cb(self, data: NavSatFix):
        print("start pos registered", data.latitude, data.longitude)
        self.start_lat = data.latitude
        self.start_lon = data.longitude

    def gpsstartori_cb(self, data: Float64):
        print("start ori registered", data.data)
        self.start_ori = data.data

    def gpssensor_cb(self, data: NavSatFix):
        if (self.start_lat != None and self.start_lon != None):
            if ((not math.isnan(self.start_lat)) and (not math.isnan(self.start_lon))):
                if(data.position_covariance[0] < 2 and data.position_covariance[4] < 2):
                    x, y = self.gps2odom(data.latitude, data.longitude)
                    dist = self.dist(self.previous_x, self.previous_y, x, y)
                    if (dist < 1000):
                        msg = Odometry()
                        msg.header.stamp = data.header.stamp
                        msg.pose.pose.position.x = x
                        msg.pose.pose.position.y = y
                        msg.pose.pose.position.z = 0

                        msg.pose.covariance[0] = data.position_covariance[0]
                        msg.pose.covariance[7] = data.position_covariance[4]
                        msg.pose.covariance[14] = data.position_covariance[8]

                        self.odom_pub.publish(msg)
                        self.previous_x = x
                        self.previous_y = y

    def gpsfoxglove_cb(self, data: NavSatFix):
        if (self.start_lat != None and self.start_lon != None):
            if ((not math.isnan(self.start_lat)) and (not math.isnan(self.start_lon))):
                x, y = self.gps2odom(data.latitude, data.longitude)
                msg = Odometry()
                msg.header.stamp = self.latest_stamp
                msg.pose.pose.position.x = x
                msg.pose.pose.position.y = y
                msg.pose.pose.position.z = 0

                msg.pose.covariance[0] = 0.0000001
                msg.pose.covariance[7] = 0.0000001
                msg.pose.covariance[14] = 0.0000001

                self.odom_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('gps2odom_node')
    GPS2Odom()
    rospy.spin()    