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
import math
from geometry_msgs.msg import AccelStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan, Imu
from std_msgs.msg import Empty, String, Header
from fusion_msgs.msg import sensorFusionMsg, controllerFusionMsg, monitorStatusMsg
from dynamic_reconfigure.client import Client

import numpy as np
import matplotlib.pyplot as plt
from my_sm import start_sm

class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['SETUP_DONE', 'FINISH'],
                             input_keys=['counter_in','acc_results', 'cam_results', 'odom_results', 'imu_results', 'lidar_results', 'mic_results', 'result_cum', 'results_', 'x_array'],
                             output_keys=['counter_out','acc_results', 'cam_results', 'odom_results', 'imu_results', 'lidar_results', 'mic_results', 'result_cum', 'results_', 'x_array'])
        #rospy.spin()
        self.acc_client = Client("accelerometer_process", timeout=3, config_callback=self.callback)
        self.cam_client = Client("vision_utils_ros", timeout=3, config_callback=self.callback)
        self.odom_client = Client("odom_collisions", timeout=3, config_callback=self.callback)
        self.imu_client = Client("imu_detector", timeout=3, config_callback=self.callback)
        self.lidar_client = Client("laser_collisions", timeout=3, config_callback=self.callback)
        self.mic_client = Client("mic_collisions", timeout=3, config_callback=self.callback)

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
        self.mic_client.update_configuration({"window_size": userdata.counter_in})
        self.mic_client.update_configuration({"reset": True})

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
            userdata.results_['mic'] = userdata.mic_results

            return 'FINISH'

class Plotter(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['PLOT_DONE'],
                             input_keys=['data_in', 'x_array'])
    def execute(self, userdata):
        rospy.loginfo('Executing Plotter')
        return 'PLOT_DONE'

# define state Monitor
class Monitor(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['NEXT_MONITOR', 'END_MONITOR'],
                              input_keys=['acc_cum', 'cam_cum', 'odom_cum', 'imu_cum', 'lidar_cum', 'mic_cum', 'overall_cum', 'result_cum'],
                              output_keys=['acc_cum', 'cam_cum', 'odom_cum', 'imu_cum', 'lidar_cum', 'mic_cum', 'overall_cum', 'result_cum'])
        rospy.Subscriber("/finish_reading", String, self.fb_cb)
        rospy.Subscriber("/collision_label", Header, self.header_cb)
        rospy.Subscriber("/detector_diagnoser_node/overall_collision", monitorStatusMsg, self.diagnoser_cb, queue_size = 1000)


        sensor_id_labels = ['accelerometer_1', 'odom', 'imu_1', 'lidar_1', 'mic_1', 'cam_0']
        self.collisions_id = dict()
        self.observer_counter = dict()

        for l in sensor_id_labels:
            self.collisions_id[l] = 0
            self.observer_counter[l] = 0

        self.current_counter = 0
        self.accel_count = 0
        self.cam_count = 0
        self.odom_count = 0
        self.imu_count = 0
        self.lidar_count = 0
        self.mic_count = 0
        self.overall_count = 0
        self.ground_truth_count = 0
        self.ground_truth = list()
        self.sf_detection = list()
        self.false_positives_count = 0
        self.false_negative_count = 0

        self.delays = list()

        self.acc_cum = list()
        self.cam_cum = list()
        self.odom_cum = list()
        self.imu_cum = list()
        self.lidar_cum = list()
        self.mic_cum = list()
        self.overall_cum = list()

        self.start_time = rospy.rostime.get_rostime().to_sec()

        for i in range(10):
            rospy.Subscriber("/collisions_"+str(i), sensorFusionMsg, self.counter_cb, queue_size=100)

    def diagnoser_cb(self,msg):
        curr_time = rospy.rostime.get_rostime().to_sec()
        self.overall_count = self.overall_count + 1
        rospy.logerr("Collision Detection")
        rospy.loginfo('curr_time %s',curr_time - self.start_time)
        self.sf_detection.append(curr_time - self.start_time)

        for m in msg.observers_ids:
            self.collisions_id[m] = self.collisions_id[m] + 1

    def counter_cb(self,msg):
        if msg.msg == 2:
            self.observer_counter[msg.sensor_id.data] = self.observer_counter[msg.sensor_id.data] + 1
            self.current_counter = self.current_counter + 1

    def fb_cb(self,msg):
        #print ("CB", msg)
        self.next_bag_request = True
        self.stop_bag_request = False

        if msg.data == "NEXT_BAG":
            print ("current_counter" , self.current_counter)
            #self.current_counter = 0
        else:#FINISH
            #print ("/n")
            print ("Ground Truth Count ", self.ground_truth_count)
            print ("Average Reaction Time ", np.mean(self.delays))
            print ("False Positives ", self.false_positives_count)
            print ("False Negatives ", self.false_negative_count)
            print ("Total collisions detected by all observers: " , self.current_counter)
            print ("Collisions by Observer " , self.observer_counter)
            print ("SF Total Collisions Detected" , self.overall_count)
            print ("Participation Rate ", self.collisions_id)

            self.stop_bag_request = True

    def header_cb(self,msg):
        curr_time = rospy.rostime.get_rostime().to_sec()
        rospy.logerr("GROUND TRUTH: %s", msg.stamp.secs)
        rospy.loginfo('curr_time %s',curr_time - self.start_time)
        self.ground_truth_count = self.ground_truth_count + 1
        self.ground_truth.append(curr_time - self.start_time)

    def execute(self, userdata):

        self.next_bag_request = False
        rospy.loginfo('Executing state MONITORING')
        #rospy.sleep(20)#TODO
        self.start_time = rospy.rostime.get_rostime().to_sec()

        while not self.next_bag_request:
            pass #TODO

        rospy.sleep(0.2)
        self.next_bag_request = False

        if self.stop_bag_request:
            userdata.acc_cum.append(self.observer_counter['accelerometer_1'])
            userdata.cam_cum.append(self.observer_counter['cam_0'])
            userdata.odom_cum.append(self.observer_counter['odom'])
            userdata.imu_cum.append(self.observer_counter['imu_1'])
            userdata.lidar_cum.append(self.observer_counter['lidar_1'])
            userdata.mic_cum.append(self.observer_counter['mic_1'])

            return 'END_MONITOR'
        else:
            #print ("NEXT_MONITOR")
            print ("Ground Truths " , self.ground_truth)
            collisions_detected = len(self.sf_detection)
            print ('SF [%s]' % ', '.join(map(str, self.sf_detection)))
            print ("Collisions Detected " , collisions_detected)

            if collisions_detected > 0: # If collisions were detecteds

                for gt in self.ground_truth:
                    delay = np.abs(np.array(self.sf_detection)-gt).min() #closest collisions -> Ground Truth
                    arg_delay = np.abs(np.array(self.sf_detection)-gt).argmin() # index of the closes collision detecteds

                    print ('Closest to ', gt ," is ", delay) # Closest delay print

                    if delay < 0.2: #if delay is less than 1 second then it is considered as a true positive
                        self.delays.append(delay)
                        collisions_detected = collisions_detected - 1 #our counter decreased
                        self.sf_detection.remove(self.sf_detection[arg_delay]) # removing from the detected collsiions
                        #self.false_positives_count = self.false_positives_count + collisions_detected - 1 # all detections minus the closest are false positives
                        for  c in self.sf_detection:
                            if 0.3> np.abs(c - gt) > 0.2: # if a collision detected is less than 0.7 seconds it is considered as a false positive
                                print ("FOUND COllision Close to Ground Truth " , c - gt)
                                self.sf_detection.remove(c) # removing from the detected collsiions
                                collisions_detected = collisions_detected - 1 #our counter decreased
                    else:
                        print ("Collision NOT FOUND")
                        self.false_negative_count = self.false_negative_count + 1

            else: #The ground truth was not detected
                print ("ANY COLLISION DETECTED, false negatives added ", len(self.ground_truth))
                self.false_negative_count = self.false_negative_count + len(self.ground_truth)


            print ("FALSE POSITIVES FOUND : ", collisions_detected )
            self.false_positives_count = collisions_detected + self.false_positives_count #if best delay is bigger than 1 second then the collision was not detected

            self.ground_truth = list()
            self.sf_detection = list()
            return 'NEXT_MONITOR'


if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')
    start_sm("/home/jose/data/collisions_2702/", "collision_bags_bags_2702_", Monitor, Setup, Plotter, time_limit = 15, max_bag_file = 50)
