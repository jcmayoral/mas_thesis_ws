import rospy
from MyStatics.RealTimePlotter import RealTimePlotter
from geometry_msgs.msg import AccelStamped
from dynamic_reconfigure.server import Server
from accelerometer_ros.cfg import accelerometerGaussConfig
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from FaultDetection import ChangeDetection
import matplotlib.pyplot as plt


class AccCUSUM(RealTimePlotter,ChangeDetection):
    def __init__(self, max_samples = 500, pace = 2, cusum_window_size = 10 ):
        self.data_ = []
        self.data_.append([0,0,0])
        self.step_ = []
        self.step_.append(0)
        self.i = 0
        self.msg = 0
        self.window_size = cusum_window_size
        RealTimePlotter.__init__(self,max_samples,pace)
        ChangeDetection.__init__(self,3)
        rospy.init_node("accelerometer_cusum", anonymous=True)
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
        self.addData([msg.accel.linear.x,msg.accel.linear.y, msg.accel.angular.z])

        if ( len(self.samples) > self.window_size):
            self.samples.pop(0)

        self.changeDetection(len(self.samples))
        cur = np.array(self.cum_sum, dtype = object)
        self.step_.append(self.msg)
        self.data_.append(cur)
        self.msg = self.msg + 1
        self.update(msg.header.seq,self.step_,self.data_)
