import rospy
from MyStatics.RealTimePlotter import RealTimePlotter
from MyStatics.GaussianPlotter import GaussPlot
from FaultDetection import ChangeDetection
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


class LaserGaussCUSUM(RealTimePlotter,ChangeDetection,GaussPlot):
    def __init__(self, max_samples = 500, pace = 2, cusum_window_size = 10 ):
        self.data_ = []
        self.data_.append([0,0,0])
        self.i = 0
        self.msg = 0
        self.window_size = cusum_window_size
        RealTimePlotter.__init__(self,max_samples,pace)
        ChangeDetection.__init__(self,10,721)
        GaussPlot.__init__(self )
        rospy.init_node("laser_detection_ros_gaus_cusum", anonymous=True)
        rospy.Subscriber("scan_unified", LaserScan, self.laserCB)
        plt.legend()
        plt.show()
        rospy.spin()
        plt.close("all")

    def laserCB(self, msg):
        while (self.i< self.window_size):
            self.addData([i for i in msg.ranges])
            self.i = self.i+1
            if len(self.samples) is self.max_samples:
                self.samples.pop(0)
            return
        self.i=0
        self.changeDetection(len(self.samples))
        cur = np.array(self.cum_sum, dtype = object)
        self.call(np.mean(self.samples, axis=0),np.var(self.samples, axis=0))
        """
        THIS IS NOT REALLY WORKING
        x1 = np.linspace(-140, 140, len(self.s_z))
        print(len(x1), len(np.sort(self.s_z)))
        plt.scatter([x1,x1,x1],np.sort(self.s_z))
        """
        x = np.linspace(0, 10, 200)
        y = np.array([i.pdf(x) for i in self.rv])
        self.update(msg.header.seq,x.tolist(),y.T.tolist())
