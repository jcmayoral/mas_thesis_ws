import rospy
from FaultDetection import ChangeDetection
from geometry_msgs.msg import AccelStamped
from sensor_msgs.msg import LaserScan
from fusion_msgs.msg import sensorFusionMsg
import numpy as np

from dynamic_reconfigure.server import Server
from laser_detection_ros.cfg import laserConfig

class FusionLaser(ChangeDetection):
    def __init__(self, cusum_window_size = 10, frame="base_link", sensor_id="laser1", threshold = 10000):
        self.data_ = []
        self.data_.append([0,0,0])
        self.i = 0
        self.msg = 0
        self.window_size = cusum_window_size
        self.frame = frame
        self.sensor_id = sensor_id
        self.threshold = threshold
        self.weight = 1.0
        ChangeDetection.__init__(self,721)
        rospy.init_node("laser_fusion", anonymous=False)
        rospy.Subscriber("/scan_unified", LaserScan, self.laserCB)
        self.pub = rospy.Publisher('collisions_3', sensorFusionMsg, queue_size=10)
        self.dyn_reconfigure_srv = Server(laserConfig, self.dynamic_reconfigureCB)
        rospy.spin()

    def dynamic_reconfigureCB(self,config, level):
        self.threshold = config["threshold"]
        self.window_size = config["window_size"]
        self.weight = config["weight"]
        return config

    def laserCB(self, msg):

        while (self.i< self.window_size):
            self.addData([i/msg.range_max for i in msg.ranges])
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
        #if any(t > self.threshold for t in cur):

        #if any(t > self.threshold for t in cur):
        print (np.sum(cur)/721)
        if np.sum(cur)/721 > self.threshold:
            msg.msg = sensorFusionMsg.ERROR

        msg.sensor_id.data = self.sensor_id
        msg.data = cur
        msg.weight = self.weight

        self.pub.publish(msg)
