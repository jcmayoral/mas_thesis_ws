import rospy
from MyStatics.RealTimePlotter import RealTimePlotter
from geometry_msgs.msg import AccelStamped
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

class Plotter(RealTimePlotter):
    def __init__(self, threshold = 1000, pace = 200):
        self.data_ = []
        self.step_ = []
        print ("Plotter Constructor Initialized")
        RealTimePlotter.__init__(self,threshold,pace)
        rospy.init_node("accelerometer_plotter", anonymous=True)
        rospy.Subscriber("accel", AccelStamped, self.accCB)
        plt.show()
        rospy.spin()
        plt.close("all")

    def accCB(self, msg):
        self.step_.append(msg.header.seq)
        self.data_.append([msg.accel.linear.x,msg.accel.linear.y, msg.accel.angular.z])
        self.update(msg.header.seq,self.step_,self.data_)
