import rospy
from MyStatics.RealTimePlotter import RealTimePlotter
from nav_msgs.msg import Odometry
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

class Plotter(RealTimePlotter):
    def __init__(self, threshold = 1000, pace = 200):
        self.data_ = []
        self.step_ = []
        print ("Plotter Constructor Initialized")
        RealTimePlotter.__init__(self,threshold,pace,True)
        #self.ax.legend("True")
        rospy.init_node("amcl_plotter", anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.amclCB)
        plt.show()
        rospy.spin()
        plt.close("all")

    def amclCB(self, msg):
        data = [msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z]
        self.step_.append(msg.header.seq)
        self.data_.append(data)
        self.update(msg.header.seq,self.step_,self.data_)
