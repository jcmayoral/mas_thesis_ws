import rospy
from MyStatics.RealTimePlotter import RealTimePlotter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from accelerometer_ros.cfg import accelerometerGaussConfig
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from FaultDetection import ChangeDetection
import matplotlib.pyplot as plt


class KalmanFilterPloter(RealTimePlotter,ChangeDetection):
    def __init__(self, max_samples = 500, pace = 2, cusum_window_size = 10 ):
        self.data_ = []
        self.data_.append([0,0,0])
        self.step_ = []
        self.step_.append(0)
        self.i = 0
        self.msg = 0
        self.current_data = Twist()
        self.window_size = cusum_window_size
        RealTimePlotter.__init__(self,max_samples,pace)
        ChangeDetection.__init__(self,3)
        rospy.init_node("controller_cusum", anonymous=True)
        self.openLoop_ = Twist()
        self.closeLoop_ = Twist()
        rospy.Subscriber("/base/twist_mux/command_navigation", Twist, self.openLoopCB)
        rospy.Subscriber("/base/odometry_controller/odometry", Odometry, self.closeLoopCB)
        self.header = 0

        plt.legend()
        plt.show()
        rospy.spin()
        plt.close("all")


    def updateData(self,msg):
        self.addData([self.current_data.linear.x, self.current_data.linear.y, self.current_data.angular.z])

        if ( len(self.samples) > self.window_size):
            self.samples.pop(0)

        self.changeDetection(len(self.samples))
        cur = np.array(self.cum_sum, dtype = object)
        self.step_.append(self.header)
        self.data_.append(cur)
        self.header = self.header + 1
        self.update(self.header,self.step_,self.data_)

    def openLoopCB(self, msg):
        self.current_data.linear.x = self.openLoop_.linear.x - self.closeLoop_.linear.x
        self.current_data.linear.y = self.openLoop_.linear.y - self.closeLoop_.linear.y
        self.current_data.angular.z = self.openLoop_.angular.z - self.closeLoop_.angular.z

        self.updateData(msg)
        self.openLoop_ = msg

    def closeLoopCB(self, msg):
        self.closeLoop_ = msg.twist.twist
