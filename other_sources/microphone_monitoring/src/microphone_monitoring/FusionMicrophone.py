import rospy
from FaultDetection import ChangeDetection
from fusion_msgs.msg import sensorFusionMsg
import numpy as np
import pyaudio

#Dynamic Reconfigure
from dynamic_reconfigure.server import Server
from microphone_monitoring.cfg import microphoneConfig

class FusionMicrophone(ChangeDetection):
    def __init__(self, cusum_window_size = 10, frame="base_link", sensor_id="microphone1", threshold = 1000, frames_number=1024):
        self.data_ = []
        self.data_.append([0,0,0])
        self.i = 0
        self.msg = 0
        self.window_size = cusum_window_size
        self.frame = frame
        self.threshold = threshold
        self.frames_number = frames_number
        self.weight = 1.0
        rospy.init_node("microphone_fusion", anonymous=False)
        ChangeDetection.__init__(self,1)

        audio = pyaudio.PyAudio()
        self.stream = audio.open(format=pyaudio.paInt16,
                            channels=1,
                            rate=44100, input=True,
                            frames_per_buffer=1024)
        self.stream.start_stream()
        r = rospy.Rate(10)
        sensor_number = rospy.get_param("~sensor_number", 0)
        self.sensor_id = rospy.get_param("~sensor_id", sensor_id)
        self.pub = rospy.Publisher('collisions_'+ str(sensor_number), sensorFusionMsg, queue_size=10)
        self.dyn_reconfigure_srv = Server(microphoneConfig, self.dynamic_reconfigureCB)

        while not rospy.is_shutdown():
            self.run()
            #r.sleep()

        self.stream.stop_stream()

        rospy.spin()

    def dynamic_reconfigureCB(self,config, level):
        self.threshold = config["threshold"]
        self.window_size = config["window_size"]
        self.weight = config["weight"]
        return config


    def run(self):
        data = self.stream.read(self.frames_number)
        amplitude = np.fromstring(data, np.int16)

        if self.i< self.window_size:
            self.addData(amplitude)
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
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame
        msg.window_size = self.window_size


        #Detecting Collisions
        suma = np.sum(np.array(self.cum_sum, dtype = object))
        print (suma)
        if suma > self.threshold:
            msg.msg = sensorFusionMsg.ERROR

        msg.sensor_id.data = self.sensor_id
        msg.data = cur
        msg.weight = self.weight
        self.pub.publish(msg)
