import rospy
from FaultDetection import ChangeDetection
from std_msgs.msg import Header
import numpy as np

from sensor_msgs.msg import Imu
#Dynamic Reconfigure
from dynamic_reconfigure.server import Server
from imu_ros.cfg import imuLabelerConfig
import math
#(0, {'defaultSampleRate': 44100.0, 'defaultLowOutputLatency': 0.008684807256235827, 'defaultLowInputLatency': 0.008684807256235827, 'maxInputChannels': 1L, 'structVersion': 2L, 'hostApi': 0L, 'index': 0, 'defaultHighOutputLatency': 0.034829931972789115, 'maxOutputChannels': 2L, 'name': u'USB Audio Device: - (hw:1,0)', 'defaultHighInputLatency': 0.034829931972789115})


class LabelerIMU(ChangeDetection):
    def __init__(self, cusum_window_size = 10, frame="base_link", sensor_id="labeler1", threshold = 1000, CHUNK=1024):
        self.data_ = np.zeros(6)
        self.i = 0
        self.msg = 0
        self.window_size = cusum_window_size
        self.frame = frame
        self.threshold = threshold
        self.weight = 1.0
        self.is_disable = False
        self.is_filtered_available = False
        self.is_collision_expected = False
        self.is_over_lapping_required = False
        self.is_covariance_detector_enable = False

        rospy.init_node("imu_labeler", anonymous=False)
        ChangeDetection.__init__(self,6)

        sensor_number = rospy.get_param("~sensor_number", 0)
        self.pub = rospy.Publisher('collision_label', Header, queue_size=1)
        self.sensor_id = rospy.get_param("~sensor_id", sensor_id)
        self.dyn_reconfigure_srv = Server(imuLabelerConfig, self.dynamic_reconfigureCB)
        rospy.loginfo("Imu Labeler Ready")

        input_topic = rospy.get_param("~input_topic", "/imu")
        rospy.Subscriber(input_topic, Imu, self.imuCB)

        rospy.spin()

    def dynamic_reconfigureCB(self,config, level):
        self.threshold = config["threshold"]
        self.window_size = config["window_size"]
        self.is_over_lapping_required = config["overlap"]

        if config["reset"]:
            self.clear_values()
            config["reset"] = False

        return config


    def imuCB(self, msg):
        if self.is_over_lapping_required:
            self.addData([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, #]) #Just Linear For Testing
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) #Angula
            if len(self.samples) > self.window_size:
                self.samples.pop(0)

        else:
            if ( self.i < self.window_size) and len(self.samples) < self.window_size:
                self.addData([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, #]) #Just Linear For Testing
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) #Angula
                self.i = self.i+1
            else:
                self.samples.pop(0)
                return

        self.i =0

        msg = Header()

        self.changeDetection(len(self.samples))
        cur = np.array(self.cum_sum, dtype = object)
        #cur = np.append(cur, covariance)

        #Filling Message
        msg.stamp = rospy.Time.now()
        msg.frame_id = self.frame

        if not self.is_disable:
            if any(t > self.threshold for t in cur if not math.isnan(t)):
                print ("COllision")
                if not self.is_disable:
                    self.pub.publish(msg)
