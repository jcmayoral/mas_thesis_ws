import rospy
from MyStatics.RealTimePlotter import RealTimePlotter
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from FaultDetection import ChangeDetection
import matplotlib.pyplot as plt

class LaserCUSUM(RealTimePlotter,ChangeDetection):
    def __init__(self, max_samples = 500, pace = 2, cusum_window_size = 10 ):
        self.data_ = []
        self.step_ = []
        self.i = 0
        self.msg = 0
        self.window_size = cusum_window_size
        RealTimePlotter.__init__(self,max_samples,pace,False)
        ChangeDetection.__init__(self,1,1)
        rospy.init_node("laser_detection_ros_cusum", anonymous=True)
        rospy.Subscriber("/scan_unified", LaserScan, self.laserCB)
        plt.show()
        rospy.spin()
        plt.close("all")

    def laserCB(self, msg):
        print ("inside")
        while (self.i< self.window_size):
            self.addData([i/msg.range_max for i in msg.ranges])
            self.i = self.i+1
            if len(self.samples) is self.max_samples:
                self.samples.pop(0)
            return
        self.i=0
        self.changeDetection(len(self.samples))
        cur = np.array(self.cum_sum)
        cur = np.nan_to_num(cur)
        cur[np.isnan(cur)] = 0
        #for i in range(len(cur)):
        #    if (cur[i] > 1000):
        #        cur[i] = 0
        cur = np.var(cur)
        self.step_.append(self.msg)
        self.data_.append(cur)
        self.msg = self.msg + 1
        self.update(msg.header.seq,self.step_,self.data_)
