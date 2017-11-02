import rospy
from FaultDetection import ChangeDetection
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from fusion_msgs.msg import sensorFusionMsg
import numpy as np

class FusionAMCL(ChangeDetection):
    def __init__(self, cusum_window_size = 10, frame="base_link", sensor_id="laser1", threshold = 100):
        self.data_ = []
        self.data_.append([0,0,0])
        self.i = 0
        self.msg = 0
        self.window_size = cusum_window_size
        self.frame = frame
        self.sensor_id = sensor_id
        self.threshold = threshold
        ChangeDetection.__init__(self,3)
        rospy.init_node("laser_cusum", anonymous=True)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amclCB)
        self.pub = rospy.Publisher('collisions_1', sensorFusionMsg, queue_size=10)
        rospy.spin()

    def amclCB(self, msg):
        data = [msg.pose.covariance[0],msg.pose.covariance[1], msg.pose.covariance[35]]
        while (self.i< self.window_size):
            self.addData(data)
            self.i = self.i+1
            if len(self.samples) is self.window_size:
                self.samples.pop(0)
            return

        msg = sensorFusionMsg()

        self.i=0
        self.changeDetection(len(self.samples))
        cur = np.array(self.cum_sum)
        cur = np.nan_to_num(cur)
        cur[np.isnan(cur)] = 0

        #Filling Message
        msg.header.frame_id = self.frame
        msg.window_size = self.window_size

        #Detecting Collisions
        if any(t > self.threshold for t in cur):
            msg.msg = sensorFusionMsg.ERROR

        msg.sensor_id.data = self.sensor_id
        msg.data = cur
        self.pub.publish(msg)
