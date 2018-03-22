import pyaudio
import matplotlib.pyplot as plt
import numpy as np
from FaultDetection import ChangeDetection

FORMAT = pyaudio.paInt16                # We use 16bit format per sample
CHANNELS = 1
RATE = 44100
CHUNK = 1024                            # 1024bytes of data red from a buffer
RECORD_SECONDS = 5

# instantiate PyAudio (1)
audio = pyaudio.PyAudio()

stream = audio.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE, input=True,
                    frames_per_buffer=CHUNK)

stream.start_stream()
detector_ = ChangeDetection(1024)

fig = plt.figure()
s = fig.add_subplot(211)
s2 = fig.add_subplot(212)
j = 0

window_size = 10
threshold = 2000000

CumSum = []
rawData = []
step = []

for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    amplitude = np.fromstring(data, np.int16)

    if j< window_size:
        detector_.addData(amplitude)
        j = j+1
    else:
        detector_.changeDetection(1024)
        data = np.sum(np.array(detector_.cum_sum, dtype = object))
        step.append(i/int(RATE/ CHUNK))
        if data > threshold:
            print ("Collision")
        CumSum.append(data)
        detector_.samples.clear()
        j = 0

    rawData.append(amplitude)

#print (len(step), len(CumSum))
s2.plot(rawData[0])
s2.legend(["Raw Data"],loc="lower right")
#CumSum.pop(0)
#step.pop(0)
#CumSum.pop(0)
#step.pop(0)
s.plot(step[2:],CumSum[2:])
s.axhline(y=threshold,xmin=0, xmax=50, linewidth=2, color='r')
s.legend(["CUSUM", "Collision Threshold"],loc="upper left")
plt.legend(shadow=True, fancybox=True)
#fig.savefig('t.png')
plt.show()
stream.stop_stream()
stream.close()
audio.terminate()
