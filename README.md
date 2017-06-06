# mas_thesis_ws
Master Thesis WorkSpace

This is a private repository which contains repositories that might be used for my master thesis.


#Instructions to install
git submodule init

git submodule update

cd slam_testing

git submodule init

git submodule update

if trajectory_follower does not work:
  try 
    git clone gitgate@mas.b-it-center.de:jmayor2s/trajectory_follower.git


To add cob3 configuration files:

cd slam_testing
cd cartographer_ros
git remote add cob3 https://github.com/jcmayoral/cartographer_ros.git
git pull cob3 master

#Note#  MyRaspBerryPi repository is not ROS compatible however it installs the python package
