import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from dynamic_reconfigure.server import Server
from fusion_msgs.msg import controllerFusionMsg
import numpy as np
from MyKalmanFilter.SimpleKalmanFilter import SimpleKalmanFilter
from velocity_controller_monitoring.cfg import filterConfig

class KalmanFilterMonitor(SimpleKalmanFilter):
    def __init__(self, cusum_window_size = 10, threshold = 10 ):
        self.data_ = []
        self.data_.append([0,0,0])
        self.step_ = []
        self.step_.append(0)
        self.threshold = threshold
        self.controller_id = 'velocity_controller'
        self.i = 0
        self.msg = 0
        self.current_data = Twist()
        self.frame = 'test'
        self.window_size = cusum_window_size
        self.is_disable = False
        self.callBackFunction = self.updateThreshold

        self.initKalmanFilter()
        rospy.init_node("kalman_filter", anonymous=True)
        self.openLoop_ = Twist()
        self.closeLoop_ = Twist()
        self.pub = rospy.Publisher('filter', controllerFusionMsg, queue_size=10)
        self.dyn_reconfigure_srv = Server(filterConfig, self.dynamic_reconfigureCB)
        rospy.Subscriber("/imu/data", Imu, self.closeLoopCB)
        rospy.Subscriber("/base/odometry_controller/odometry", Odometry, self.openLoopCB)
        rospy.loginfo("Kalman Filter Initialized")
        rospy.spin()

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
        print (Q)
        SimpleKalmanFilter.__init__(self,x, A, H, R, Q, dt=1, size = 6)

    def dynamic_reconfigureCB(self,config, level):
        self.threshold = config["threshold"]
        self.window_size = config["window_size"]
        self.is_disable = config["is_disable"]

        if config["reset"]: #TODO
            config["reset"] = False

        return config


    def updateThreshold(self,msg):
        Z = np.array([np.fabs(self.current_data.linear.x),
                np.fabs(self.current_data.linear.y),
                np.fabs(self.current_data.angular.z)]).reshape(3,1)
        self.runFilter(Z)
        #data = self.getInnovationFunction().flatten()
        data = self.getEstimatedState()
        print (data, data.shape)
        self.publishMsg(data.flatten())

    def closeLoopCB(self, msg):
        self.current_data.linear.x = self.openLoop_.linear.x - msg.linear_acceleration.x
        self.current_data.linear.y = self.openLoop_.linear.y - msg.linear_acceleration.y
        self.current_data.angular.z = self.openLoop_.angular.z - msg.angular_velocity.z

        self.callBackFunction(msg)

    def openLoopCB(self, msg):
        self.openLoop_ = msg.twist.twist

    def publishMsg(self,data):
        output_msg = controllerFusionMsg()
        #Filling Message
        output_msg.header.frame_id = self.frame
        output_msg.window_size = self.window_size
        #print ("Accelerations " , x,y,z)
        output_msg.controller_id.data = self.controller_id

        if any(t > self.threshold for t in data):
            rospy.logwarn("IGNORE")
            output_msg.mode = controllerFusionMsg.IGNORE

        output_msg.header.stamp = rospy.Time.now()

        if not self.is_disable:
            self.pub.publish(output_msg)
