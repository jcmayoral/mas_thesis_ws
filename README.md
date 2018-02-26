# mas_thesis_ws
Master Thesis WorkSpace

This is a private repository which contains repositories that might be used for my master thesis.

# Libraries

## BMC

BMC is a third part function collection which contain several standard fucntion for pattern detection. We are interested in the detect_cusum function however we suggest to install the complete library to proceed with the proposed research.

cd BMC
sudo python setup.py install

**Note** A testing function is provided in MyRaspBerryPi package.

## MyRaspberryPi

if (RaspBerryPi is used): <br />
   cd sensors/accelerometer/Adafruit_Python_ADXL345 <br />
   sudo python setup.py install <br />

cd Fault_Detection/FaultDetection <br />
sudo python setup.py install <br />

cd statics/MyStatics <br />
sudo python setup.py install <br />

### Testing

FaultDetection library provides some tests available in testing folder.

* Print in screen accelerometer readings :   python plotOnScreen.py
* Test BMC-version CUSUM :   python plotBMCDetectCUSUM.py
* Sample Stochastic Detection algorithm :   python plotMathAccelerometeri.py

## VisionUtils

This package provide a ROS independent package to test the collision detection within the integrated camera. It includes a qtcreator pro file if qtcreator IDE will be use to modified the project. It requires the version of OpenCV 3.2. To install run:

cd VisionUtils<br />
mkdir build && cd build<br />
cmake ..<br />
make<br />
sudo make install<br />

### Testing
type collision_detection in any terminal

## Collisions_stuff

We suggest an extension of the original move_base package to handle collisions being heavily based on the original navigation stack.

## vision_utils_ros

The master branch is designed for OpenCV 2.4.8 available by default in the indigo distribution of ROS. If kinetic version is required, checkout the kinetic_devel branch.
