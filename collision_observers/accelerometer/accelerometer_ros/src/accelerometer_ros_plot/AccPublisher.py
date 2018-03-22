import rospy
import Adafruit_ADXL345
from geometry_msgs.msg import AccelStamped

class AccPublisher:

    def __init__(self):
        rospy.init_node('acceleometer_ros', anonymous=True)
        self.acc_pub = rospy.Publisher("/accel", AccelStamped, queue_size = 1)
        self.accel = Adafruit_ADXL345.ADXL345()
        self.seq = 0
        print ("Publisher Node Initialized")

    def run(self, frequency = 0):
        while not rospy.is_shutdown():
            x, y, z = self.accel.read()
            msg = AccelStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "accel_frame"
            msg.accel.linear.x = x
            msg.accel.linear.y = y
            msg.accel.angular.z = z
            msg.header.seq = self.seq
            self.seq = self.seq + 1
            self.acc_pub.publish(msg)
