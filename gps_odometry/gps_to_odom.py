#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
import math
import utm


class GPS_TO_ODOM(Node):
    def __init__(self):
        super().__init__('gps_to_odom')

        self.gnss_subscriber=self.create_subscription(Vector3Stamped,'/filter/positionlla',self.gnss_callback,10)
        self.imu_subscriber=self.create_subscription(Imu,'/imu/data',self.imu_callback,10)

        self.odom_publisher=self.create_publisher(Odometry,'/odometry/gps',10)
        timer_period=0.25
        self.timer = self.create_timer(timer_period, self.odom_callback) 

        self.i=0
        self.j=0

        self.wait_for_fix=0
        self.wait_for_fix_time=250

        self.gnss=None
        self.utm=None

        self.temp_yaw=None
        self.initial_yaw=None
        self.yaw=None
        self.orientation=None
        self.utm=None
        self.initial_utm=None

    def gnss_callback(self,gnss):
        latitude=gnss.vector.x
        longitude=gnss.vector.y

        if self.j==0:
            self.temp_utm=utm.from_latlon(latitude,longitude)
            self.j=1

        self.initial_utm=self.temp_utm
        self.utm=utm.from_latlon(latitude,longitude)

    def imu_callback(self,imu):
        orientation=imu.orientation
        w=orientation.w
        x=orientation.x
        y=orientation.y
        z=orientation.z
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = w * w + x * x - y * y - z * z

        if self.i==0:
            self.temp_yaw=math.atan2(siny_cosp, cosy_cosp)
            self.i=1

        self.initial_yaw=self.temp_yaw
        self.yaw=math.atan2(siny_cosp, cosy_cosp)
        self.orientation=orientation
        self.angular_velocity=round(imu.angular_velocity.z,4)

    def pose(self):

        self.initial_x=(self.initial_utm[0]*math.cos(self.initial_yaw))-(self.initial_utm[1]*math.sin(self.initial_yaw))
        self.initial_y=(self.initial_utm[0]*math.sin(self.initial_yaw))+(self.initial_utm[1]*math.cos(self.initial_yaw))

        self.temp_x=(self.utm[0]*math.cos(self.initial_yaw))-(self.utm[1]*math.sin(self.initial_yaw))
        self.temp_y=(self.utm[0]*math.sin(self.initial_yaw))+(self.utm[1]*math.cos(self.initial_yaw))

        self.x=self.temp_x-self.initial_x
        self.y=self.temp_y-self.initial_y
        coordinates=[self.x,self.y]
        return coordinates

    def odom_callback(self):
        odom=Odometry()
        pose=Pose()
        print("initial yaw=", self.initial_yaw)
        print("yaw=", self.yaw)

        print(self.wait_for_fix)
        if self.wait_for_fix<(self.wait_for_fix_time*4):
            self.wait_for_fix+=1


        elif self.initial_utm!=None:
            coordinates=self.pose()
            x=coordinates[0]
            y=coordinates[1]

            print(coordinates)

            pose.position.x=x
            pose.position.y=y
            odom.pose.pose=pose

            odom.child_frame_id="imu_link"
            odom.header.frame_id="odom"
            self.odom_publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)

    gps_odom_publisher=GPS_TO_ODOM()
    rclpy.spin(gps_odom_publisher)

    gps_odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()