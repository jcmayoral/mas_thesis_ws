import rospy
from MyStatics.RealTimePlotter import RealTimePlotter
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from FaultDetection import ChangeDetection
import matplotlib.pyplot as plt

class OdomMonitoring(RealTimePlotter,ChangeDetection):
    def __init__(self, max_samples = 500, pace = 2, cusum_window_size = 10 ):
        self.data_ = []
        self.step_ = []
        self.i = 0
        self.msg = 0
        self.window_size = cusum_window_size
        RealTimePlotter.__init__(self,max_samples,pace,True)
        ChangeDetection.__init__(self,3)
        rospy.init_node("odom_monitoring_cusum", anonymous=True)
        rospy.Subscriber("/base/odometry_controller/odometry", Odometry, self.odomCB, queue_size=1000)
        plt.show()
        rospy.spin()
        plt.close("all")

    def odomCB(self, msg):
        data = [msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z]
        self.addData(data)

        if ( len(self.samples) > self.window_size):
            self.samples.pop(0)

        self.changeDetection(len(self.samples))
        cur = np.array(self.cum_sum)
        cur = np.nan_to_num(cur)
        #cur[np.isnan(cur)] = 0
        #for i in range(len(cur)):
        #    if (cur[i] > 1000):
        #        cur[i] = 0
        #cur = np.var(cur)
        print(cur)
        self.step_.append(self.msg)
        self.data_.append(cur)
        self.msg = self.msg + 1
        self.update(msg.header.seq,self.step_,self.data_)
