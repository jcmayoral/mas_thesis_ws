import rospy
from FaultDetection import ChangeDetection
from std_msgs.msg import Header
import numpy as np
import pyaudio

#Dynamic Reconfigure
from dynamic_reconfigure.server import Server
from microphone_monitoring.cfg import microphoneConfig


#(0, {'defaultSampleRate': 44100.0, 'defaultLowOutputLatency': 0.008684807256235827, 'defaultLowInputLatency': 0.008684807256235827, 'maxInputChannels': 1L, 'structVersion': 2L, 'hostApi': 0L, 'index': 0, 'defaultHighOutputLatency': 0.034829931972789115, 'maxOutputChannels': 2L, 'name': u'USB Audio Device: - (hw:1,0)', 'defaultHighInputLatency': 0.034829931972789115})


class LabelerMicrophone(ChangeDetection):
    def __init__(self, cusum_window_size = 10, frame="base_link", sensor_id="microphone1", threshold = 1000, CHUNK=1024):
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

        rospy.init_node("microphone_labeler", anonymous=False)
        ChangeDetection.__init__(self,1)

        audio = pyaudio.PyAudio()
        for i in range(audio.get_device_count()):
             dev = audio.get_device_info_by_index(i)
             print(i,dev)

	device_index = 0
        dev = audio.get_device_info_by_index(device_index)
        print dev["defaultSampleRate"]
        self.stream = audio.open(format=pyaudio.paInt16,
                            channels=1,
                            input_device_index = device_index,
                            rate=(int)(dev["defaultSampleRate"]), input=True,
                            frames_per_buffer=self.CHUNK)
        self.stream.start_stream()
        r = rospy.Rate(10)
        sensor_number = rospy.get_param("~sensor_number", 0)
        self.sensor_id = rospy.get_param("~sensor_id", sensor_id)
        self.pub = rospy.Publisher('collision_label', Header, queue_size=1)
        self.dyn_reconfigure_srv = Server(microphoneConfig, self.dynamic_reconfigureCB)

        while not rospy.is_shutdown():
            self.run()
            #r.sleep()

        self.stream.stop_stream()

        rospy.spin()

    def reset_publisher(self):
        pass

    def dynamic_reconfigureCB(self,config, level):
        self.threshold = config["threshold"]
        self.window_size = config["window_size"]
        self.weight = config["weight"]
        self.is_disable = config["is_disable"]
        self.sensor_number = config["detector_id"]
        self.reset_publisher()

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

        msg = Header()

        self.i=0
        self.changeDetection(len(self.samples))
        cur = np.array(self.cum_sum)
        cur = np.nan_to_num(cur)
        cur[np.isnan(cur)] = 0

        #Filling Message
        msg.stamp = rospy.Time.now()
        msg.frame_id = self.frame


        #Detecting Collisions
        suma = np.sum(np.array(self.cum_sum, dtype = object))
        print (suma)
        if suma > self.threshold:
            print ("COllision")
            if not self.is_disable:
                self.pub.publish(msg)
