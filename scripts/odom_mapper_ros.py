#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class OdometryMapper:
    def __init__(self):
        rospy.init_node('odometry_mapper', anonymous=True)
        self.sub = rospy.Subscriber('/base/odometry_controller/odometry', Odometry, self.odom_Cb)
        self.Odom = Odometry()
        data_x_to_plot = list()
        data_y_to_plot = list()

        rate = rospy.Rate(0.5)
        rospy.loginfo('Starting Node    ')

        while not rospy.is_shutdown():
            data_x_to_plot.append(self.Odom.pose.pose.position.x)
            data_y_to_plot.append(self.Odom.pose.pose.position.y)
            rate.sleep()

        plt.scatter(data_x_to_plot, data_y_to_plot)
        plt.title('Odometry Positions Map on Odometry Frame')
        plt.xlabel('X Coordinate [m]')
        plt.ylabel('Y Coordinate [m]')
        plt.legend()
        plt.show()


    def odom_Cb(self,msg):
        self.Odom = msg


if __name__ == '__main__':
    OdometryMapper()
