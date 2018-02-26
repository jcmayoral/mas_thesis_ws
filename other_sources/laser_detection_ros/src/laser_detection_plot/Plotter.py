import rospy
from MyStatics.RealTimePlotter import RealTimePlotter
from sensor_msgs.msg import LaserScan
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

class Plotter(RealTimePlotter):
    def __init__(self, threshold = 1000, pace = 200):
        self.data_ = []
        self.step_ = []
        print ("Plotter Constructor Initialized")
        RealTimePlotter.__init__(self,threshold,pace,False)
        self.ax.legend("False")
        rospy.init_node("laser_plotter", anonymous=True)
        rospy.Subscriber("/scan_unified", LaserScan, self.laserCB)
        plt.show()
        rospy.spin()
        rospy.sleep(2)
        plt.close("all")

    def laserCB(self, msg):
        max_value = msg.range_max
        self.step_.append(msg.header.seq)
        self.data_.append([i/max_value for i in msg.ranges])
        self.update(msg.header.seq,self.step_,self.data_)
