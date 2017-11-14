import rospy
from FaultDetection import ChangeDetection
from fusion_msgs.msg import sensorFusionMsg
import numpy as np
import pyaudio

#Dynamic Reconfigure
from dynamic_reconfigure.server import Server
from microphone_monitoring.cfg import microphoneConfig

class FusionMicrophone(ChangeDetection):
    def __init__(self, cusum_window_size = 10, frame="base_link", sensor_id="microphone1", threshold = 1000000):
        self.data_ = []
        self.data_.append([0,0,0])
        self.i = 0
        self.msg = 0
        self.window_size = cusum_window_size
        self.frame = frame
        self.sensor_id = sensor_id
        self.threshold = threshold
        rospy.init_node("microphone_fusion", anonymous=True)
        ChangeDetection.__init__(self,1)

        audio = pyaudio.PyAudio()
        self.stream = audio.open(format=pyaudio.paInt16,
                            channels=1,
                            rate=44100, input=True,
                            frames_per_buffer=1024)
        self.stream.start_stream()
        r = rospy.Rate(10)
        self.pub = rospy.Publisher('collisions_1', sensorFusionMsg, queue_size=10)

        self.dyn_reconfigure_srv = Server(microphoneConfig, self.dynamic_reconfigureCB)

        while not rospy.is_shutdown():
            self.run()
            #r.sleep()

        self.stream.stop_stream()

        rospy.spin()

    def dynamic_reconfigureCB(self,config, level):
        for k, v in config.iteritems():
            print (k, ":", v)
        self.threshold = config["threshold"]
        return config


    def run(self):
        data = self.stream.read(1024)
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
        #print (suma)
        if suma > self.threshold:
            msg.msg = sensorFusionMsg.ERROR

        msg.sensor_id.data = self.sensor_id
        msg.data = cur
        self.pub.publish(msg)
