import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from fusion_msgs.msg import sensorFusionMsg
import numpy as np
from FaultDetection import ChangeDetection
from velocity_controller_monitoring.cfg import filterConfig


class CollisionFilter(ChangeDetection):
    def __init__(self, cusum_window_size = 10, threshold = 10 ):
        self.data_ = []
        self.data_.append([0,0,0])
        self.step_ = []
        self.step_.append(0)
        self.threshold = threshold
        self.sensor_id = 'master'
        self.i = 0
        self.msg = 0
        self.current_data = Twist()
        self.frame = 'test'
        self.window_size = cusum_window_size
        ChangeDetection.__init__(self,3)
        rospy.init_node("controller_cusum", anonymous=True)
        self.openLoop_ = Twist()
        self.closeLoop_ = Twist()
        self.pub = rospy.Publisher('filter', sensorFusionMsg, queue_size=10)
        self.dyn_reconfigure_srv = Server(filterConfig, self.dynamic_reconfigureCB)
        rospy.Subscriber("/base/twist_mux/command_navigation", Twist, self.openLoopCB)
        rospy.Subscriber("/base/odometry_controller/odometry", Odometry, self.closeLoopCB)
        rospy.spin()

    def dynamic_reconfigureCB(self,config, level):
        self.threshold = config["threshold"]
        self.window_size = config["window_size"]

        if config["reset"]:
            self.clear_values()
            config["reset"] = False
        return config

    def updateData(self,msg):
        self.addData([self.current_data.linear.x, self.current_data.linear.y, self.current_data.angular.z])

        if ( len(self.samples) > self.window_size):
            self.samples.pop(0)

        self.changeDetection(len(self.samples))
        cur = np.array(self.cum_sum, dtype = object)
        self.publishMsg(cur)

    def openLoopCB(self, msg):
        self.current_data.linear.x = self.openLoop_.linear.x - self.closeLoop_.linear.x
        self.current_data.linear.y = self.openLoop_.linear.y - self.closeLoop_.linear.y
        self.current_data.angular.z = self.openLoop_.angular.z - self.closeLoop_.angular.z

        self.updateData(msg)
        self.openLoop_ = msg

    def closeLoopCB(self, msg):
        self.closeLoop_ = msg.twist.twist

    def publishMsg(self,data):
        output_msg = sensorFusionMsg()
        #Filling Message
        output_msg.header.frame_id = self.frame
        output_msg.window_size = self.window_size
        #print ("Accelerations " , x,y,z)

        if any(t > self.threshold for t in data):
            output_msg.msg = sensorFusionMsg.ERROR

        output_msg.header.stamp = rospy.Time.now()
        output_msg.sensor_id.data = self.sensor_id
        output_msg.data = data
        output_msg.weight = self.weight
        self.pub.publish(output_msg)
