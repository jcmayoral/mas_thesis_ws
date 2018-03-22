import rospy
from MyStatics.RealTimePlotter import RealTimePlotter
from MyStatics.GaussianPlotter import GaussPlot
from FaultDetection import ChangeDetection
from geometry_msgs.msg import AccelStamped
from dynamic_reconfigure.server import Server
from accelerometer_ros.cfg import accelerometerGaussConfig
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


class AccGaussCUSUM(RealTimePlotter,ChangeDetection,GaussPlot):
    def __init__(self, max_samples = 500, pace = 2, cusum_window_size = 10 ):
        self.data_ = []
        self.data_.append([0,0,0])
        self.i = 0
        self.msg = 0
        self.window_size = cusum_window_size
        RealTimePlotter.__init__(self,max_samples,pace)
        ChangeDetection.__init__(self,3)
        GaussPlot.__init__(self )
        rospy.init_node("accelerometer_gauss_cusum", anonymous=True)
        rospy.Subscriber("accel", AccelStamped, self.accCB)
        self.dyn_reconfigure_srv = Server(accelerometerGaussConfig, self.dynamic_reconfigureCB)

        plt.legend()
        plt.show()
        rospy.spin()
        plt.close("all")

    def dynamic_reconfigureCB(self,config, level):
        self.window_size = config["window_size"]
        return config

    def accCB(self, msg):
        while (self.i< self.window_size):
            self.addData([msg.accel.linear.x,msg.accel.linear.y, msg.accel.angular.z])
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
        x = np.linspace(-140, 140, 200)
        y = np.array([i.pdf(x) for i in self.rv])
        self.update(msg.header.seq,x.tolist(),y.T.tolist())
