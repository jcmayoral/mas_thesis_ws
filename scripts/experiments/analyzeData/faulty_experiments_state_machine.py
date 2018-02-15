#!/usr/bin/env python
"""
This state machine is designed to read an "n" number of rosbag files given
a common string name on such a way that all rosbag files must share that name
followed by a number, i.e. "test" with rosbag files "test1.bag", "test2.bag"...
"testX.bag".

Once a rosbag file had finished the next is executed.

Created by: Jose Mayoral
"""
#import roslib; roslib.load_manifest('smach_tutorials')
from __future__ import print_function
import rospy
import smach
import smach_ros
import time
import math
from geometry_msgs.msg import AccelStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan, Imu
from std_msgs.msg import Empty, String, Header
from fusion_msgs.msg import sensorFusionMsg, controllerFusionMsg
from dynamic_reconfigure.client import Client
import numpy as np
import matplotlib.pyplot as plt
from my_sm import start_sm

from  common_states import MyBagReader, RestartReader

class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['SETUP_DONE', 'FINISH'],
                             input_keys=['counter_in','acc_results', 'cam_results', 'odom_results', 'imu_results', 'result_cum', 'results_', 'x_array'],
                             output_keys=['counter_out','acc_results', 'cam_results', 'odom_results', 'imu_results', 'result_cum', 'results_', 'x_array'])
        #rospy.spin()
        self.acc_client = Client("accelerometer_process", timeout=3, config_callback=self.callback)
        self.cam_client = Client("vision_utils_ros", timeout=3, config_callback=self.callback)
        self.odom_client = Client("odom_collisions", timeout=3, config_callback=self.callback)
        self.imu_client = Client("imu_detector", timeout=3, config_callback=self.callback)
        self.lidar_client = Client("laser_collisions", timeout=3, config_callback=self.callback)

        self.is_first_time = True

        rospy.sleep(0.2)


    def callback(self,config):
        #print (config)
        pass
        #rospy.loginfo("Config set to {double_param}, {int_param}, {double_param}, ".format(**config))

    def execute(self, userdata):
        rospy.loginfo('Executing SETUP')
        self.acc_client.update_configuration({"window_size": 20})#userdata.counter_in})
        self.acc_client.update_configuration({"reset": True})
        self.odom_client.update_configuration({"window_size": 20})#userdata.counter_in})
        self.odom_client.update_configuration({"reset": True})
        self.imu_client.update_configuration({"window_size": 20})#userdata.counter_in})
        self.imu_client.update_configuration({"reset": True})
        self.lidar_client.update_configuration({"window_size": 20})#userdata.counter_in})
        self.lidar_client.update_configuration({"reset": True})

        #SURF Version
        self.cam_client.update_configuration({"matching_threshold":  userdata.counter_in * 0.01})
        self.cam_client.update_configuration({"mode": 0})
        self.cam_client.update_configuration({"reset": True})

        rospy.sleep(0.5)
        if self.is_first_time:#userdata.counter_in < 75: # Window SIZe Define max TODO
            userdata.x_array.append(userdata.counter_in)
            userdata.counter_out = userdata.counter_in + 5
            self.is_first_time = False
            return 'SETUP_DONE'
        else:
            userdata.results_['accel'] = userdata.acc_results
            userdata.results_['cam'] = userdata.cam_results
            userdata.results_['odom'] = userdata.odom_results
            userdata.results_['imu'] = userdata.imu_results
            userdata.results_['lidar'] = userdata.lidar_results

            return 'FINISH'

class Plotter(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['PLOT_DONE'],
                             input_keys=['data_in', 'x_array'])
    def execute(self, userdata):
        rospy.loginfo('Executing SETUP')
        pass
        """
        #Accelerometer Plot
        plt.figure()
        data = np.array(userdata.data_in['accel'])
        x_array = userdata.x_array
        plt.plot(x_array,data[:,0], label='Accelerometer Threshold X')
        plt.plot(x_array,data[:,1], label='Accelerometer Threshold Y')
        plt.plot(x_array,data[:,2], label='Accelerometer Threshold Z')
        plt.xlabel("Window Size")
        plt.ylabel("Maximum Threshold Detected")
        plt.legend()
        plt.title('Accelerometer on Motion Thresholding') # subplot 211 title
        plt.savefig('accelerometer_motion_threshold.png') # TODO

        #Camera Plot
        plt.figure()
        data = np.array(userdata.data_in['cam'])
        plt.plot(np.asarray(x_array) * 0.01, data[:,0], label='RGB Camera Thresholds')
        plt.xlabel("Matching Threshold Variation")
        plt.ylabel("Maximum Threshold Detected")
        plt.title('Camera on Motion Thresholding') # subplot 211 title
        plt.legend()
        plt.savefig('camera_motion_threshold.png') # TODO

        plt.figure()
        data = np.array(userdata.data_in['odom'])
        x_array = userdata.x_array
        plt.plot(x_array,data[:,0], label='Odometry Threshold X')
        plt.plot(x_array,data[:,1], label='Odometry Threshold Y')
        plt.plot(x_array,data[:,2], label='Odometry Threshold Z')
        plt.xlabel("Window Size")
        plt.ylabel("Maximum Threshold Detected")
        plt.legend()
        plt.title('Odometry Speeds on Motion Thresholding') # subplot 211 title
        plt.savefig('odometry_motion_threshold.png') # TODO

        plt.figure()
        data = np.array(userdata.data_in['imu'])
        x_array = userdata.x_array
        plt.plot(x_array,data[:,0], label='Linear Acceleration X')
        plt.plot(x_array,data[:,1], label='Linear Acceleration Y')
        plt.plot(x_array,data[:,2], label='Linear Acceleration Z')
        plt.plot(x_array,data[:,3], label='Angular Velocity X')
        plt.plot(x_array,data[:,4], label='Angular Velocity Y')
        plt.plot(x_array,data[:,5], label='Angular Velocity Z')
        plt.xlabel("Window Size")
        plt.ylabel("Maximum Threshold Detected")
        plt.legend()
        plt.title('IMU Speeds on Motion Thresholding') # subplot 211 title
        plt.savefig('imu_motion_threshold.png') # TODO

        f = open( 'file', 'w' )
        for key, value in userdata.data_in.iteritems():
        #for item in userdata.data_in:
            f.write("'{0}'/n".format(key))
            f.write("'{0}'/n".format(value))
        f.close()
        #plt.show()
        rospy.sleep(0.5)
        """
        return 'PLOT_DONE'

# define state Monitor
class Monitor(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['NEXT_MONITOR', 'END_MONITOR'],
                              input_keys=['acc_cum', 'cam_cum', 'odom_cum', 'imu_cum', 'lidar_cum', 'result_cum'],
                              output_keys=['acc_cum', 'cam_cum', 'odom_cum', 'imu_cum', 'lidar_cum', 'result_cum'])
        rospy.Subscriber("/finish_reading", String, self.fb_cb)
        rospy.Subscriber("/collision_label", Header, self.header_cb)

        self.current_counter = 0
        self.accel_count = 0
        self.cam_count = 0
        self.odom_count = 0
        self.imu_count = 0
        self.lidar_count = 0


        self.acc_cum = list()
        self.cam_cum = list()
        self.odom_cum = list()
        self.imu_cum = list()
        self.lidar_cum = list()

        self.start_time = rospy.rostime.get_rostime().to_sec()

        for i in range(10):
            rospy.Subscriber("/collisions_"+str(i), sensorFusionMsg, self.counter_cb, queue_size=100)

    def counter_cb(self,msg):

        if msg.msg == 2:
            rospy.logwarn("collision_detected by %s with data %s" , msg.sensor_id , msg.data)

            if msg.sensor_id.data == "accelerometer_1":
                self.accel_count = self.accel_count + 1
            if msg.sensor_id.data == "cam_0":
                self.cam_count = self.cam_count + 1
            if msg.sensor_id.data == "odom":
                self.odom_count = self.odom_count + 1
            if msg.sensor_id.data == "imu_1":
                self.imu_count = self.imu_count + 1
            if msg.sensor_id.data == "lidar_1":
                self.lidar_count = self.lidar_count + 1

            self.current_counter = self.current_counter + 1

    def fb_cb(self,msg):
        #print ("CB", msg)
        self.next_bag_request = True
        self.stop_bag_request = False

        if msg.data == "NEXT_BAG":
            self.current_counter = 0
        else:#FINISH
            print ("current_counter" , self.current_counter)
            print ("accel_count" , self.accel_count , " collisions detected " , self.current_counter)
            print ("cam_count" , self.cam_count , " collisions detected " , self.current_counter)
            print ("odom_count" , self.odom_count , " collisions detected " , self.current_counter)
            print ("imu_count" , self.imu_count , " collisions detected " , self.current_counter)
            print ("lidar_count" , self.lidar_count , " collisions detected " , self.current_counter)


            self.stop_bag_request = True

    def header_cb(self,msg):
        curr_time = rospy.rostime.get_rostime().to_sec()
        print (end="\n")
        if (15 > curr_time - self.start_time > 5): #TO AVOID INITIAL FALSE
            rospy.logerr("GROUND TRUTH: %s", msg.stamp.secs)

    def execute(self, userdata):

        self.next_bag_request = False
        rospy.loginfo('Executing state MONITORING')
        #rospy.sleep(20)#TODO

        while not self.next_bag_request:
            pass #TODO

        rospy.sleep(0.2)
        self.next_bag_request = False

        if self.stop_bag_request:
            userdata.acc_cum.append(self.accel_count)
            userdata.cam_cum.append(self.cam_count)
            userdata.odom_cum.append(self.odom_count)
            userdata.imu_cum.append(self.imu_count)
            userdata.imu_cum.append(self.lidar_count)


            return 'END_MONITOR'
        else:
            #print ("NEXT_MONITOR")
            self.start_time = rospy.rostime.get_rostime().to_sec()
            return 'NEXT_MONITOR'


if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')
    start_sm("/home/jose/data/", "collision_bags_bags", Monitor, Setup, Plotter, time_limit = 15, max_bag_file = 50)
