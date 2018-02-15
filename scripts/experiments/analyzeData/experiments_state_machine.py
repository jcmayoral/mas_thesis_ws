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
import rosbag
import math
from geometry_msgs.msg import AccelStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Empty, String
from fusion_msgs.msg import sensorFusionMsg
from dynamic_reconfigure.client import Client
import numpy as np
import matplotlib.pyplot as plt
from my_sm import start_sm

class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['SETUP_DONE', 'FINISH'],
                             input_keys=['counter_in','acc_results', 'cam_results', 'odom_results', 'lidar_results', 'result_cum', 'results_', 'x_array'],
                             output_keys=['counter_out','acc_results', 'cam_results', 'odom_results', 'lidar_results', 'result_cum', 'results_', 'x_array'])
        #rospy.spin()
        self.acc_client = Client("accelerometer_process", timeout=3, config_callback=self.callback)
        self.cam_client = Client("vision_utils_ros", timeout=3, config_callback=self.callback)
        self.odom_client = Client("odom_collisions", timeout=3, config_callback=self.callback)
        self.lidar_client = Client("laser_collisions", timeout=3, config_callback=self.callback)

        rospy.sleep(0.2)


    def callback(self,config):
        #print (config)
        pass
        #rospy.loginfo("Config set to {double_param}, {int_param}, {double_param}, ".format(**config))

    def execute(self, userdata):
        rospy.loginfo('Executing SETUP')
        self.acc_client.update_configuration({"window_size": userdata.counter_in})
        self.acc_client.update_configuration({"reset": True})
        self.odom_client.update_configuration({"window_size": userdata.counter_in})
        self.odom_client.update_configuration({"reset": True})
        self.lidar_client.update_configuration({"window_size": userdata.counter_in})
        self.lidar_client.update_configuration({"reset": True})


        #SURF Version
        self.cam_client.update_configuration({"matching_threshold":  userdata.counter_in * 0.01})
        self.cam_client.update_configuration({"mode": 0})
        self.cam_client.update_configuration({"reset": True})

        rospy.sleep(0.5)
        if userdata.counter_in < 3: # Window SIZe Define max TODO
            userdata.x_array.append(userdata.counter_in)
            userdata.counter_out = userdata.counter_in + 5
            return 'SETUP_DONE'
        else:
            userdata.results_['accel'] = userdata.acc_results
            userdata.results_['cam'] = userdata.cam_results
            userdata.results_['odom'] = userdata.odom_results
            userdata.results_['lidar'] = userdata.lidar_results
            return 'FINISH'

class Plotter(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['PLOT_DONE'],
                             input_keys=['data_in', 'x_array'])
    def execute(self, userdata):
        rospy.loginfo('Executing SETUP')

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


        plt.figure() #TODO
        data = np.array(userdata.data_in['lidar'])
        x_array = userdata.x_array
        plt.plot(x_array,data, label='Lidar Threshold X')
        plt.xlabel("Window Size")
        plt.ylabel("Maximum Threshold Detected")
        plt.legend()
        plt.title('Lidar on Motion Thresholding') # subplot 211 title
        plt.savefig('lidar_motion_threshold.png') # TODO

        f = open( 'file', 'w' )
        for key, value in userdata.data_in.iteritems():
        #for item in userdata.data_in:
            f.write("'{0}'/n".format(key))
            f.write("'{0}'/n".format(value))
        f.close()
        #plt.show()
        rospy.sleep(0.5)
        return 'PLOT_DONE'

# define state Monitor
class Monitor(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['NEXT_MONITOR', 'END_MONITOR'],
                              input_keys=['acc_cum', 'cam_cum', 'odom_cum', 'lidar_cum', 'result_cum'],
                              output_keys=['acc_cum', 'cam_cum', 'odom_cum', 'lidar_cum', 'result_cum'])
        rospy.Subscriber("/finish_reading", String, self.fb_cb)
        self.current_counter = 0
        self.accel_thr = list()
        self.cam_thr = list()
        self.odom_thr = list()
        self.lidar_thr = list()

        self.acc_cum = list()
        self.cam_cum = list()
        self.odom_cum = list()
        self.lidar_cum = list()

        for i in range(10):
            rospy.Subscriber("/collisions_"+str(i), sensorFusionMsg, self.threshold_cb, queue_size=100)

    def threshold_cb(self,msg):
        if msg.sensor_id.data == "accelerometer_1":
            self.accel_thr.append(msg.data)
        if msg.sensor_id.data == "cam1":
            self.cam_thr.append(msg.data)
        if msg.sensor_id.data == "odom_1":
            self.odom_thr.append(msg.data)
        if msg.sensor_id.data == "lidar_1":
            self.lidar_thr.append(msg.data)
        #print (msg.data)

    def fb_cb(self,msg):
        #print ("CB", msg)
        self.next_bag_request = True
        self.stop_bag_request = False

        if msg.data == "NEXT_BAG":
            self.current_counter = 0
        else:#FINISH
            #print ("current_counter" , self.current_counter)
            print ("accel_thr" , np.nanmax(self.accel_thr, axis=0) , " number of samples " , len(self.accel_thr))
            print ("cam_thr" , np.nanmax(self.cam_thr, axis=0), " number of samples " , len(self.cam_thr))
            print ("odom_thr" , np.nanmax(self.odom_thr, axis=0), " number of samples " , len(self.odom_thr))
            print ("lidar_thr" , np.nanmax(self.lidar_thr, axis=0), " number of samples " , len(self.lidar_thr))

            self.stop_bag_request = True

    def execute(self, userdata):

        self.next_bag_request = False
        rospy.loginfo('Executing state MONITORING')
        #rospy.sleep(20)#TODO

        while not self.next_bag_request:
            pass #TODO

        rospy.sleep(0.2)
        self.next_bag_request = False

        if self.stop_bag_request:
            userdata.acc_cum.append(np.nanmax(self.accel_thr, axis=0))
            userdata.cam_cum.append(np.nanmax(self.cam_thr, axis=0))
            userdata.odom_cum.append(np.nanmax(self.odom_thr, axis=0))
            userdata.lidar_cum.append(np.nanmax(self.lidar_thr, axis=0))


            del self.accel_thr[:]
            del self.cam_thr[:]
            del self.odom_thr[:]
            del self.lidar_thr[:]

            return 'END_MONITOR'
        else:
            #print ("NEXT_MONITOR")
            return 'NEXT_MONITOR'


if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')
    start_sm("/home/jose/ROS/thesis_ws/my_ws/rosbag_test/cob3/cob3-test-2301/", "cob3-attempt-2301-", Monitor, Setup, Plotter)
