import rospy
from MyStatics.RealTimePlotter import RealTimePlotter
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

class Plotter(RealTimePlotter):
    def __init__(self, threshold = 1000, pace = 200):
        self.data_ = []
        self.step_ = []
        print ("Plotter Constructor Initialized")
        super().__init__(threshold,pace,True)
        #self.ax.legend("True")
        rospy.init_node("amcl_plotter", anonymous=True)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amclCB)
        plt.show()
        rospy.spin()
        plt.close("all")

    def amclCB(self, msg):
        data = [msg.pose.covariance[0],msg.pose.covariance[1], msg.pose.covariance[35]]
        self.step_.append(msg.header.seq)
        self.data_.append(data)
        self.update(msg.header.seq,self.step_,self.data_)
