# mas_thesis_ws
Master Thesis WorkSpace

git submodule init


git submodule update

cd slam_testing

git submodule init

git submodule update

if trajectory_follower does not work:
  try 
    git clone gitgate@mas.b-it-center.de:jmayor2s/trajectory_follower.git


To add cob3 configuration files

cd slam_testing
cd cartographer_ros
git remote add cob3 https://github.com/jcmayoral/cartographer_ros.git
git pull cob3 master
