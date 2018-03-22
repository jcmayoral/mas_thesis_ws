import rospy
from CollisionSensorTemplates.SensorFusion import CollisionFusionSensor
from geometry_msgs.msg import AccelStamped
from fusion_msgs.msg import sensorFusionMsg
import numpy as np

#Dynamic Reconfigure
from dynamic_reconfigure.server import Server
from accelerometer_ros.cfg import accelerometerConfig

class AccFusionTemplate(CollisionFusionSensor):
    def __init__(self):
        CollisionFusionSensor.__init__(self,
                              number_elements=6,
                              window_size = 10,
                              frame = "acc_frame",
                              sensor_id = "imu",
                              threshold = 60,
                              node = "acc_collision",
                              sensor_type = AccelStamped,
                              topic_name = "collisions_",
                              sensor_number = 0,
                              config_type = accelerometerConfig)

    def update(self,msg):
        self.addData([msg.accel.linear.x,msg.accel.linear.y, msg.accel.angular.z])
