import rospy
from audio_common_msgs.msg import AudioData
import numpy as np
import pyaudio

#(0, {'defaultSampleRate': 44100.0, 'defaultLowOutputLatency': 0.008684807256235827, 'defaultLowInputLatency': 0.008684807256235827, 'maxInputChannels': 1L, 'structVersion': 2L, 'hostApi': 0L, 'index': 0, 'defaultHighOutputLatency': 0.034829931972789115, 'maxOutputChannels': 2L, 'name': u'USB Audio Device: - (hw:1,0)', 'defaultHighInputLatency': 0.034829931972789115})

class MyAudioCapture():
    def __init__(self):
        rospy.init_node("my_audio_capture", anonymous=False)

        self.audio = pyaudio.PyAudio()
        print (self.audio.get_device_count())
        for i in range(self.audio.get_device_count()):
             dev = self.audio.get_device_info_by_index(i)
        self.device_index = 0
        self.CHUNK = 1024
        self.device = self.audio.get_device_info_by_index(self.device_index)
        self.stream = self.audio.open(format=pyaudio.paInt16,
                            channels=1,
                            input_device_index = self.device_index,
                            rate=(int)(self.device["defaultSampleRate"]), input=True,
                            frames_per_buffer=self.CHUNK)
        self.stream.start_stream()
        r = rospy.Rate(10)
        self.pub = rospy.Publisher('audio',AudioData, queue_size=10)

        while not rospy.is_shutdown():
            self.run()
            #r.sleep()
        self.stream.stop_stream()
        rospy.spin()

    def run(self):
        try:
            data = self.stream.read(self.CHUNK)
        except IOError as ex:
            if ex[1] != pyaudio.paInputOverflowed:
                raise
            print ("ERROR")
            return

        amplitude = np.fromstring(data, np.uint8)
        print type(amplitude)
        msg = AudioData()
        msg.data = amplitude.tolist()

        self.pub.publish(msg)
