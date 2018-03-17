import rospy
from MyStatics.RealTimePlotter import RealTimePlotter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

class ControllerMonitoring(RealTimePlotter):
    def __init__(self, threshold = 1000, pace = 200):
        self.data_ = []
        self.step_ = []
        print ("Plotter Constructor Initialized")
        rospy.init_node("velocity_controller_monitoring")
        RealTimePlotter.__init__(self,threshold,pace,True)
        #self.ax.legend("True")
        rospy.Subscriber("/base/twist_mux/command_navigation", Twist, self.openLoopCB)
        rospy.Subscriber("/base/odometry_controller/odometry", Odometry, self.closeLoopCB)
        self.openLoop_ = Twist()
        self.closeLoop_ = Twist()
        self.count = 0
        plt.show()

    def updateData(self):
        self.step_.append(self.count)
        self.data_.append([self.openLoop_.linear.x - self.closeLoop_.linear.x,
                           self.openLoop_.linear.y - self.closeLoop_.linear.y,
                           self.openLoop_.angular.z - self.closeLoop_.angular.z])
        self.update(self.count,self.step_,self.data_)

    def openLoopCB(self, msg):
        self.count = self.count + 1
        self.openLoop_ = msg

    def closeLoopCB(self, msg):
        self.closeLoop_ = msg.twist.twist
        self.updateData()
