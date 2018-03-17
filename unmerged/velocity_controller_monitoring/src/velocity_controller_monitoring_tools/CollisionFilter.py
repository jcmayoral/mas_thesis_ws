import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from fusion_msgs.msg import controllerFusionMsg
import numpy as np
from FaultDetection import ChangeDetection
from velocity_controller_monitoring.cfg import filterConfig


class CollisionFilter(ChangeDetection):
    def __init__(self, threshold = 10 ):
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
        self.is_disable = False
        ChangeDetection.__init__(self,3)
        rospy.init_node("controller_cusum", anonymous=True)
        self.openLoop_ = Twist()
        self.closeLoop_ = Twist()
        self.pub = rospy.Publisher('filter', controllerFusionMsg, queue_size=1)
        self.dyn_reconfigure_srv = Server(filterConfig, self.dynamic_reconfigureCB)
        rospy.Subscriber("/base/twist_mux/command_navigation", Twist, self.openLoopCB)
        rospy.Subscriber("/base/odometry_controller/odometry", Odometry, self.closeLoopCB)
        rospy.spin()

    def dynamic_reconfigureCB(self,config, level):
        self.threshold = config["threshold"]
        self.is_disable = config["is_disable"]

        return config


    def updateThreshold(self,msg):
        data = [np.fabs(self.current_data.linear.x),
                np.fabs(self.current_data.linear.y),
                np.fabs(self.current_data.angular.z)]
        self.publishMsg(data)


    def openLoopCB(self, msg):
        self.openLoop_ = msg

    def closeLoopCB(self, msg):
        self.closeLoop_ = msg.twist.twist
        self.current_data.linear.x = self.openLoop_.linear.x - self.closeLoop_.linear.x
        self.current_data.linear.y = self.openLoop_.linear.y - self.closeLoop_.linear.y
        self.current_data.angular.z = self.openLoop_.angular.z - self.closeLoop_.angular.z

        self.updateThreshold(msg)


    def publishMsg(self,data):
        output_msg = controllerFusionMsg()
        #Filling Message
        output_msg.header.frame_id = self.frame
        output_msg.window_size = 1
        #print ("Accelerations " , x,y,z)
        output_msg.controller_id.data = self.controller_id
        output_msg.header.stamp = rospy.Time.now()
        print (data)

        if any(abs(t) > self.threshold for t in data):
            rospy.logwarn("IGNORE")
            output_msg.mode = controllerFusionMsg.IGNORE
        if not self.is_disable:
            self.pub.publish(output_msg)
            rospy.sleep(0.1)
