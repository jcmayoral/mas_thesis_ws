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
        self.delta = 1
        self.offset_x = 0
        self.offset_y = 0
        self.offset_z = 0
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
        dt = self.delta
        x = np.array([0,0,0,0,0,0]).reshape((6,1)) # Initial state
        P = np.eye(6) * 10 # Initial Uncertanty
        A = np.array(([1,0,0,dt,0,0],
                      [0,1,0,0,dt,0],
                      [0,0,1,0,0,dt],
                      [0,0,0,1,0,0],
                      [0,0,0,0,1,0],
                      [0,0,0,0,0,1])) # Dynamic Matrix Function

        H = np.array(([1,0,0,0,0,0],
                      [0,1,0,0,0,0],
                      [0,0,1,0,0,0],
                      [0,0,0,1,0,0],
                      [0,0,0,0,1,0],
                      [0,0,0,0,0,1])) # Measurement Function
        #Q = np.array(([1/4*np.power(dt,4), 1/4*np.power(dt,4), 1/4*np.power(dt,4),1/2*np.power(dt,3), 1/2*np.power(dt,3), 1/2*np.power(dt,3)],
        # 	      [1/4*np.power(dt,4), 1/4*np.power(dt,4), 1/4*np.power(dt,4), 1/2*np.power(dt,3), 1/2*np.power(dt,3), 1/2*np.power(dt,3)],
        #   	      [1/4*np.power(dt,4), 1/4*np.power(dt,4), 1/4*np.power(dt,4), 1/2*np.power(dt,3), 1/2*np.power(dt,3), 1/2*np.power(dt,3)],
        # 	      [1/2*np.power(dt,3), 1/2*np.power(dt,3), 1/2*np.power(dt,3), np.power(dt,2), np.power(dt,2), np.power(dt,2)],
        # 	      [1/2*np.power(dt,3), 1/2*np.power(dt,3), 1/2*np.power(dt,3) ,np.power(dt,2), np.power(dt,2), np.power(dt,2)],
        # 	      [1/2*np.power(dt,3), 1/2*np.power(dt,3), 1/2*np.power(dt,3) ,np.power(dt,2), np.power(dt,2), np.power(dt,2)])) # Process Noise Covariance

        R =  np.eye(6)
        Q = np.zeros(6)
        SimpleKalmanFilter.__init__(self,x, A, H, R, Q, dt=dt, size = 6)

    def reset_publisher(self):
        self.pub = rospy.Publisher('collisions_'+ str(self.sensor_number), sensorFusionMsg, queue_size=10)

    def dynamic_reconfigureCB(self,config, level):
        self.threshold = config["threshold"]
        self.is_disable = config["is_disable"]
        self.sensor_number = config["detector_id"]
        self.mode = config["mode_selector"]
        self.delta = config["delta"]
        self.offset_x = config["offset_x"]
        self.offset_y = config["offset_y"]
        self.offset_z = config["offset_z"]

        self.initKalmanFilter()

        self.reset_publisher()

        if config["reset"]: #TODO
            self.setInitialState(np.array([0,0,0,0,0,0]).reshape((6,1)))
            config["reset"] = False

        return config

    def updateThreshold(self,msg):
        Z = np.array([self.openLoop_.linear.x,
                      self.openLoop_.linear.y,
                      self.openLoop_.angular.z,
                      self.closeLoop_.linear_acceleration.x - self.offset_x,
                      self.closeLoop_.linear_acceleration.y - self.offset_y,
                      self.closeLoop_.angular_velocity.z - self.offset_z]).reshape(6,1)
        self.runFilter(Z)

        if self.mode is 0:
            data = self.getInnovationFunction()
        else:
            data = self.getEstimatedState()

        print (data, data.shape)
        self.publishMsg(data.flatten())

    def closeLoopCB(self, msg):
        self.current_data.linear.x = self.openLoop_.linear.x - msg.linear_acceleration.x - 10.5
        self.current_data.linear.y = self.openLoop_.linear.y - msg.linear_acceleration.y - 5.4
        self.current_data.angular.z = self.openLoop_.angular.z - msg.angular_velocity.z + 14.3
        self.closeLoop_ = msg
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

        #data[0] = data[0] - 10.8 #TODO
        #data[1] = data[1] - 6.67#TODO
        #data[2] = data[2] + 14.02#TODO
        output_msg.data = data
        output_msg.header.stamp = rospy.Time.now()

        if not self.is_disable:
            self.pub.publish(output_msg)
