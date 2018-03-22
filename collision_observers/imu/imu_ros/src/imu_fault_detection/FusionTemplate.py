import rospy
from CollisionSensorTemplates.SensorFusion import CollisionFusionSensor
from geometry_msgs.msg import AccelStamped
from sensor_msgs.msg import Imu
from fusion_msgs.msg import sensorFusionMsg
import numpy as np

#Dynamic Reconfigure
from dynamic_reconfigure.server import Server
from imu_ros.cfg import imuConfig

class IMUFusionTemplate(CollisionFusionSensor):
    def __init__(self):
        CollisionFusionSensor.__init__(self,
                              number_elements=6,
                              window_size = 10,
                              frame = "imu_frame",
                              sensor_id = "imu",
                              threshold = 60,
                              node = "imu_collision",
                              sensor_type = Imu,
                              topic_name = "/imu/data",
                              sensor_number = 50,
                              config_type = imuConfig)

    def updateData(self,msg):
        self.current_measure =[msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, #]) #Just Linear For Testing
                              msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z] #Angular
