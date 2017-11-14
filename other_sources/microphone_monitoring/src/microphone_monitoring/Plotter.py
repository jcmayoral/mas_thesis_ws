import rospy
from MyStatics.RealTimePlotter import RealTimePlotter
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import pyaudio
#import _thread

class Plotter(RealTimePlotter):
    def __init__(self, threshold = 100, pace = 10):
        self.data_ = []
        self.step_ = []
        print ("Plotter Constructor Initialized")
        RealTimePlotter.__init__(self,threshold,pace,False)
        #self.ax.legend("True")
        rospy.init_node("microphone_plotter", anonymous=False)
        audio = pyaudio.PyAudio()


        self.stream = audio.open(format=pyaudio.paInt16,
                            channels=1,
                            rate=44100, input=True,
                            frames_per_buffer=1024)
        self.stream.start_stream()
        self.i = 0
        #_thread.start_new_thread( self.read, () )
        r = rospy.Rate(10)
        plt.show(block=False)

        while not rospy.is_shutdown():
            self.read()
            r.sleep()
        self.stream.stop_stream()
        plt.close("all")

    def read(self):
        data = self.stream.read(1024)
        amplitude = np.fromstring(data, np.int16)
        self.step_.append(self.i)
        self.i = self.i + 1
        self.data_.append(amplitude)
        self.update(self.i,self.step_,self.data_)
