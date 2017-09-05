# mas_thesis_ws
Master Thesis WorkSpace

This is a private repository which contains repositories that might be used for my master thesis.

# Libraries

## BMC

BMC is a third part function collection which contain several standard fucntion for pattern detection. We are interested in the detect_cusum function however we suggest to install the complete library to proceed with the proposed research.

cd BMC
sudo python3 setup.py install 

**Note** A testing function is provided in MyRaspBerryPi package.

## MyRaspBerryPi 

cd sensing/accelerometer<br />

if (RaspBerryPi is used): <br />
   cd Adafruit_Python_ADXL345 <br />
   sudo python3 setup.py install <br />
   cd ..<br />

cd Fault_Tolerant_Accelerometer <br />
sudo python3 setup.py install <br />

### Testing
cd Fault_Tolerant_Accelerometer <br />
cd testing

* Print in screen accelerometer readings :   python3 plotOnScreen.py
* Test BMC-version CUSUM :   python3 plotBMCDetectCUSUM.py
* Sample Stochastic Detection algorithm :   python3 plotMathAccelerometeri.py

## VisionUtils

This package provide a ROS independent package to test the collision detection within the integrated camera. It includes a qtcreator pro file if qtcreator IDE will be use to modified the project. It requires the version of OpenCV 3.2. To install run:

cd VisionUtils<br />
mkdir build && cd build<br />
cmake ..<br />
make<br />
sudo make install<br />

### Testing
type qt_vision in any terminal

## Navigation

We suggest an extension of the original move_base package to handle collisions. In case you intend to use it:

cd navigation<br />
git checkout collisions<br />


