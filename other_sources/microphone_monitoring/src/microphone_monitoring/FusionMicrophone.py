import rospy
from FaultDetection import ChangeDetection
from fusion_msgs.msg import sensorFusionMsg
from audio_common_msgs.msg import AudioData
from CollisionSensorTemplates.SensorFusion import CollisionFusionSensor
import numpy as np
import pyaudio

#Dynamic Reconfigure
from dynamic_reconfigure.server import Server
from microphone_monitoring.cfg import microphoneConfig

#(0, {'defaultSampleRate': 44100.0, 'defaultLowOutputLatency': 0.008684807256235827, 'defaultLowInputLatency': 0.008684807256235827, 'maxInputChannels': 1L, 'structVersion': 2L, 'hostApi': 0L, 'index': 0, 'defaultHighOutputLatency': 0.034829931972789115, 'maxOutputChannels': 2L, 'name': u'USB Audio Device: - (hw:1,0)', 'defaultHighInputLatency': 0.034829931972789115})

class FusionMicrophone(ChangeDetection):
    def __init__(self, cusum_window_size = 10, frame="base_link", sensor_id="microphone1", threshold = 1000, CHUNK=1):
        self.data_ = []
        self.data_.append([0,0,0])
        self.i = 0
        self.msg = 0
        self.window_size = cusum_window_size
        self.frame = frame
        self.threshold = threshold
        self.CHUNK = CHUNK
        self.weight = 1.0
        self.is_disable = False

        rospy.init_node("microphone_fusion", anonymous=False)
        ChangeDetection.__init__(self,1)

        self.audio = pyaudio.PyAudio()
        for i in range(self.audio.get_device_count()):
             dev = self.audio.get_device_info_by_index(i)
        self.device_index = 0
        print(i,dev)
        self.device = self.audio.get_device_info_by_index(self.device_index)
        self.stream = self.audio.open(format=pyaudio.paInt16,
                            channels=1,
                            input_device_index = self.device_index,
                            rate=(int)(self.device["defaultSampleRate"]), input=True,
                            frames_per_buffer=self.CHUNK)
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

    def reset_publisher(self):
        self.pub = rospy.Publisher('collisions_'+ str(self.sensor_number), sensorFusionMsg, queue_size=10)

    def dynamic_reconfigureCB(self,config, level):
        self.threshold = config["threshold"]
        self.window_size = config["window_size"]
        self.weight = config["weight"]
        self.is_disable = config["is_disable"]
        self.sensor_number = config["detector_id"]
        self.reset_publisher()
        self.stream.stop_stream()
        while not self.stream.is_stopped:
            print "h"
        self.audio.close(self.stream)
        self.stream = self.audio.open(format=pyaudio.paInt16,
                            channels=1,
                            input_device_index = self.device_index,
                            rate=(int)(self.device["defaultSampleRate"]), input=True,
                            frames_per_buffer=(int)(config['CHUNK']))

        if config["reset"]:
            self.clear_values()
            config["reset"] = False
        return config


    def run(self):
        try:
            data = self.stream.read(self.CHUNK)
        except IOError as ex:
            if ex[1] != pyaudio.paInputOverflowed:
                raise
            print ("ERROR")
            return

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
        var = np.var(np.array(self.cum_sum, dtype = object))

        #print (suma)
        if suma > self.threshold:
            #print ("COllision")
            msg.msg = sensorFusionMsg.ERROR

        msg.sensor_id.data = self.sensor_id
        msg.data = cur
        msg.weight = self.weight
        if not self.is_disable:
            self.pub.publish(msg)



class FusionAudioCapture(CollisionFusionSensor):
    def __init__(self):
        self.__number_elements = 300
        CollisionFusionSensor.__init__(self,
                              number_elements=self.__number_elements,
                              window_size = 10,
                              frame = "mic_1",
                              sensor_id = "mic_1",
                              threshold = 1000,
                              node = "microphone_collisions",
                              sensor_type = AudioData,
                              topic_name = "/audio",
                              sensor_number = 2,
                              config_type = microphoneConfig)

    def updateData(self,msg):
        print len(msg.data)
        self.current_measure = np.fromstring(msg.data[0:self.__number_elements], np.int8)
