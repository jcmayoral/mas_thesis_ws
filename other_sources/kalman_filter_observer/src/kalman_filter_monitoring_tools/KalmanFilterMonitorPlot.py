import rospy
from MyStatics.RealTimePlotter import RealTimePlotter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from dynamic_reconfigure.server import Server
from accelerometer_ros.cfg import accelerometerGaussConfig
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from MyKalmanFilter.SimpleKalmanFilter import SimpleKalmanFilter
import matplotlib.pyplot as plt


class KalmanFilterPlotter(RealTimePlotter,SimpleKalmanFilter):
    def __init__(self, max_samples = 500, pace = 2, cusum_window_size = 10 ):
        self.data_ = []
        self.data_.append([0,0,0,0,0,0])
        self.step_ = []
        self.step_.append(0)
        self.i = 0
        self.msg = 0
        self.current_data = Twist()
        self.window_size = cusum_window_size
        RealTimePlotter.__init__(self,max_samples,pace,False)
        self.initKalmanFilter()
        rospy.init_node("controller_cusum", anonymous=True)
        self.openLoop_ = Twist()
        self.closeLoop_ = Twist()
        rospy.Subscriber("/imu/data", Imu, self.closeLoopCB)
        rospy.Subscriber("/base/odometry_controller/odometry", Odometry, self.openLoopCB)
        self.header = 0

        plt.legend()
        plt.show()
        rospy.spin()
        plt.close("all")


    def initKalmanFilter(self):
        dt = 1
        x = np.array([0,0,0,0,0,0]).reshape((6,1)) # Initial state
        P = np.eye(6) * 10 # Initial Uncertanty
        A = np.eye(6) # Transition Matrix
        A[0,3] = dt
        A[1,4] = dt
        A[2,5] = dt
        H = np.array(([0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1])) # Measurement Function
        R = np.array(([10,0,0],[0,10,0],[0,0,10])) # measurement noise covariance
        Q = np.array(([1/4*np.power(dt,4), 1/4*np.power(dt,4), 1/4*np.power(dt,4),1/2*np.power(dt,3), 1/2*np.power(dt,3), 1/2*np.power(dt,3)],
        	      [1/4*np.power(dt,4), 1/4*np.power(dt,4), 1/4*np.power(dt,4), 1/2*np.power(dt,3), 1/2*np.power(dt,3), 1/2*np.power(dt,3)],
          	      [1/4*np.power(dt,4), 1/4*np.power(dt,4), 1/4*np.power(dt,4), 1/2*np.power(dt,3), 1/2*np.power(dt,3), 1/2*np.power(dt,3)],
        	      [1/2*np.power(dt,3), 1/2*np.power(dt,3), 1/2*np.power(dt,3), np.power(dt,2), np.power(dt,2), np.power(dt,2)],
        	      [1/2*np.power(dt,3), 1/2*np.power(dt,3), 1/2*np.power(dt,3) ,np.power(dt,2), np.power(dt,2), np.power(dt,2)],
        	      [1/2*np.power(dt,3), 1/2*np.power(dt,3), 1/2*np.power(dt,3) ,np.power(dt,2), np.power(dt,2), np.power(dt,2)])) # Process Noise Covariance
        SimpleKalmanFilter.__init__(self,x, A, H, R, Q, dt=1, size = 6)

    def updateData(self):
        Z = np.array([np.fabs(self.current_data.linear.x),
                np.fabs(self.current_data.linear.y),
                np.fabs(self.current_data.angular.z)]).reshape(3,1)
        self.runFilter(Z)
        #data = self.getInnovationFunction().flatten()
        data = self.getEstimatedState()
        self.step_.append(self.header)
        self.data_.append(data.flatten())
        self.header = self.header + 1
        self.update(self.header,self.step_,self.data_)

    def openLoopCB(self, msg):
        self.openLoop_ = msg.twist.twist
        self.updateData()

    def closeLoopCB(self, msg):
        self.current_data.linear.x = self.openLoop_.linear.x - msg.linear_acceleration.x
        self.current_data.linear.y = self.openLoop_.linear.y - msg.linear_acceleration.y
        self.current_data.angular.z = self.openLoop_.angular.z - msg.angular_velocity.z
