import numpy as np
import matplotlib.pyplot as plt


x = np.arange(2,75,5)
data = np.array([146.46031189, 526.5378418, 719.746521, 877.02825928, 980.95404053, 1060.71240234, 1113.99511719, 1153.0637207, 1185.11499023, 1206.82714844, 1246.4675293, 1336.64575195, 1317.92614746, 1336.84313965, 1356.78967285])



plt.ylabel("Maximum Threshold")
plt.xlabel("Matching Thresholdi Value")
plt.title("Camera SURF Matching Collision Detection Thresholding")
plt.plot(x,data)
plt.show()
plt.savefig("Camera_motion_thresholding.png");
