import rospy
from MyStatics.RealTimePlotter import RealTimePlotter
from sensor_msgs.msg import Imu
from dynamic_reconfigure.server import Server
from imu_ros.cfg import imuGaussConfig
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from FaultDetection import ChangeDetection
import matplotlib.pyplot as plt

class ImuCUSUM(RealTimePlotter,ChangeDetection):
    def __init__(self, max_samples = 500, pace = 10, cusum_window_size = 20 ):
        self.data_ = []
        self.data_.append([0,0,0,0,0,0])
        self.step_ = []
        self.step_.append(0)
        self.i = 0
        self.msg = 0
        self.window_size = cusum_window_size
        RealTimePlotter.__init__(self,max_samples,pace)
        ChangeDetection.__init__(self,6)
        rospy.init_node("imu_cusum", anonymous=True)
        input_topic = rospy.get_param("~input_topic", '/imu/data')
        rospy.Subscriber(input_topic, Imu, self.imuCB)
        self.dyn_reconfigure_srv = Server(imuGaussConfig, self.dynamic_reconfigureCB)

        plt.legend()
        plt.show()
        rospy.spin()
        plt.close("all")

    def dynamic_reconfigureCB(self,config, level):
        self.window_size = config["window_size"]
        return config

    def imuCB(self, msg):
        if (self.i< self.window_size):
            self.addData([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, #]) #Just Linear For Testing
                          msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) #Angular
            self.i = self.i+1
            if len(self.samples) is self.max_samples:
                self.samples.pop(0)
            return
        self.i=0
        self.changeDetection(len(self.samples))
        cur = np.array(self.cum_sum, dtype = object)
        self.step_.append(self.msg)
        self.data_.append(cur)
        self.msg = self.msg + 1
        self.update(msg.header.seq,self.step_,self.data_)
