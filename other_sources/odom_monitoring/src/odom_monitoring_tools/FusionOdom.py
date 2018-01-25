import rospy
from CollisionSensorTemplates.SensorFusion import CollisionFusionSensor
from nav_msgs.msg import Odometry
from fusion_msgs.msg import sensorFusionMsg
import numpy as np

#Dynamic Reconfigure
from dynamic_reconfigure.server import Server
from odom_monitoring.cfg import odomConfig

class OdomFusion(CollisionFusionSensor):
    def __init__(self):
        CollisionFusionSensor.__init__(self,
                              number_elements=6,
                              window_size = 10,
                              frame = "odom_frame",
                              sensor_id = "odom",
                              threshold = 60,
                              node = "odom_collision",
                              sensor_type = Odometry,
                              topic_name = "/base/odometry_controller/odometry",
                              sensor_number = 0,
                              config_type = odomConfig)

    def update(self,msg):
        self.addData([msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z])
