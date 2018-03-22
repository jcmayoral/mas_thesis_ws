import rospy
from MyStatics.RealTimePlotter import RealTimePlotter
from sensor_msgs.msg import Imu
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

class Plotter(RealTimePlotter):
    def __init__(self, threshold = 1000, pace = 200, input_topic='/imu'):
        self.data_ = []
        self.step_ = []
        print ("Plotter Constructor Initialized")
        RealTimePlotter.__init__(self,threshold,pace)
        rospy.init_node("imu_plotter", anonymous=True)
        rospy.Subscriber(input_topic, Imu, self.imuCB)
        plt.show()
        rospy.spin()
        plt.close("all")

    def imuCB(self, msg):
        self.step_.append(msg.header.seq)
        self.data_.append([msg.linear_acceleration.x,msg.linear_acceleration.y, msg.linear_acceleration.z, #Linear
                           msg.angular_velocity.x,msg.angular_velocity.y, msg.angular_velocity.z]) #Angular
        self.update(msg.header.seq,self.step_,self.data_)
