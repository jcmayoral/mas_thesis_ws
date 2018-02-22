import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from dynamic_reconfigure.server import Server
from fusion_msgs.msg import sensorFusionMsg
import numpy as np
from MyKalmanFilter.SimpleKalmanFilter import SimpleKalmanFilter
from kalman_filter_observer.cfg import kalmanfilterConfig

class KalmanFilterMonitor(SimpleKalmanFilter):
    def __init__(self, cusum_window_size = 10, threshold = 10, sensor_id='kalman1'):
        self.data_ = []
        self.data_.append([0,0,0])
        self.step_ = []
        self.step_.append(0)
        self.threshold = threshold
        self.sensor_id = sensor_id
        self.i = 0
        self.msg = 0
        self.mode = 0
        self.current_data = Twist()
        self.frame = 'test'
        self.window_size = cusum_window_size
        self.is_disable = False
        self.callBackFunction = self.updateThreshold

        self.initKalmanFilter()
        rospy.init_node("kalman_filter", anonymous=False)
        self.openLoop_ = Twist()
        self.closeLoop_ = Twist()

        sensor_number = rospy.get_param("~sensor_number", 0)
        self.sensor_id = rospy.get_param("~sensor_id", sensor_id)
        self.pub = rospy.Publisher('collisions_'+ str(sensor_number), sensorFusionMsg, queue_size=10)
        self.dyn_reconfigure_srv = Server(kalmanfilterConfig, self.dynamic_reconfigureCB)
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
        SimpleKalmanFilter.__init__(self,x, A, H, R, Q, dt=1, size = 6)

    def reset_publisher(self):
        self.pub = rospy.Publisher('collisions_'+ str(self.sensor_number), sensorFusionMsg, queue_size=10)

    def dynamic_reconfigureCB(self,config, level):
        self.threshold = config["threshold"]
        self.is_disable = config["is_disable"]
        self.sensor_number = config["detector_id"]
        self.mode = config["mode_selector"]

        self.reset_publisher()

        if config["reset"]: #TODO
            self.setInitialState(np.array([0,0,0,0,0,0]).reshape((6,1)))
            config["reset"] = False

        return config

    def updateThreshold(self,msg):
        Z = np.array([np.fabs(self.current_data.linear.x),
                np.fabs(self.current_data.linear.y),
                np.fabs(self.current_data.angular.z)]).reshape(3,1)
        self.runFilter(Z)

        if self.mode is 0:
            data = self.getInnovationFunction()
        else:
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
        output_msg = sensorFusionMsg()
        #Filling Message
        output_msg.header.frame_id = self.frame
        output_msg.window_size = self.window_size
        #print ("Accelerations " , x,y,z)
        output_msg.sensor_id.data = self.sensor_id

        #if any(t > self.threshold for t in data[3:5]):
        print data
        if any(t for t in data) > self.threshold:
            rospy.logwarn("Collision")
            output_msg.msg = sensorFusionMsg.ERROR

        data[0] = data[0] - 10.8 #TODO
        data[1] = data[1] - 6.67#TODO
        data[2] = data[2] + 0.02#TODO
        output_msg.data = data
        output_msg.header.stamp = rospy.Time.now()

        if not self.is_disable:
            self.pub.publish(output_msg)
