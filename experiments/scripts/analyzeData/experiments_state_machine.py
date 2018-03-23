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
from audio_common_msgs.msg import AudioData
from dynamic_reconfigure.client import Client
import numpy as np
import matplotlib.pyplot as plt
from my_sm import start_sm

class Setup(smach.State):
    def __init__(self, max_window_size = 75,step=5):
        smach.State.__init__(self,
                             outcomes=['SETUP_DONE', 'FINISH'],
                             input_keys=['counter_in','acc_results', 'cam_results', 'odom_results', 'lidar_results', 'mic_results','imu_results', 'result_cum', 'results_', 'x_array'],
                             output_keys=['counter_out','acc_results', 'cam_results', 'odom_results', 'lidar_results', 'mic_results','imu_results','result_cum', 'results_', 'x_array'])
        #rospy.spin()
        self.step = step
        self.max_window_size = max_window_size
        #self.acc_client = Client("accelerometer_process", timeout=3, config_callback=self.callback)
        self.cam_client = Client("vision_utils_ros_android", timeout=3, config_callback=self.callback)
        #self.odom_client = Client("odom_collisions", timeout=3, config_callback=self.callback)
        self.lidar_client = Client("laser_collisions", timeout=3, config_callback=self.callback)
        self.mic_client = Client("mic_collisions", timeout=3, config_callback=self.callback)
        self.imu_client = Client("imu_collision_detection", timeout=3, config_callback=self.callback)

        rospy.sleep(0.2)


    def callback(self,config):
        #print (config)
        pass
        #rospy.loginfo("Config set to {double_param}, {int_param}, {double_param}, ".format(**config))

    def execute(self, userdata):
        rospy.loginfo('Executing SETUP')
        #self.acc_client.update_configuration({"window_size": userdata.counter_in})
        #self.acc_client.update_configuration({"reset": True})
        #self.odom_client.update_configuration({"window_size": userdata.counter_in})
        #self.odom_client.update_configuration({"reset": True})
        self.lidar_client.update_configuration({"window_size": userdata.counter_in})
        self.lidar_client.update_configuration({"reset": True})
        self.mic_client.update_configuration({"window_size": userdata.counter_in})
        self.mic_client.update_configuration({"reset": True})
        self.imu_client.update_configuration({"window_size": userdata.counter_in})
        self.imu_client.update_configuration({"reset": True})


        #SURF Version
        self.cam_client.update_configuration({"matching_threshold":  userdata.counter_in * 0.01})
        self.cam_client.update_configuration({"mode": 0})
        self.cam_client.update_configuration({"reset": True})

        rospy.sleep(0.5)
        if userdata.counter_in < self.max_window_size: # Window SIZe Define max TODO
            print ("RESTART")
            userdata.x_array.append(userdata.counter_in)
            userdata.counter_out = userdata.counter_in + self.step
            return 'SETUP_DONE'

        else:
            userdata.results_['accel'] = userdata.acc_results
            userdata.results_['cam'] = userdata.cam_results
            userdata.results_['odom'] = userdata.odom_results
            userdata.results_['lidar'] = userdata.lidar_results
            userdata.results_['mic'] = userdata.mic_results
            userdata.results_['imu'] = userdata.imu_results
            return 'FINISH'

class Plotter(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['PLOT_DONE'],
                             input_keys=['data_in', 'x_array'])
    def execute(self, userdata):

        rospy.loginfo('Executing Plotter')
        #Accelerometer Plot
        print (userdata.data_in)
        data = userdata.data_in['accel']
        x_array = userdata.x_array

        if len(data['std']) > 0:
            plt.figure()
            yerr=[npdata["max"] - data["mean"], data["mean"] - data["min"]]
            plt.xlim(-0.1, x_array[-1]+1)
            plt.errorbar(x_array, np.array(data["mean"]), yerr=0.2, fmt='ok', lw=3)
            #plt.errorbar(x_array,np.array(data["mean"]), datayerr=yerr, fmt='.k', ecolor='gray', lw=1)
            plt.xlabel("Window Size")
            plt.ylabel("CUSUM Error")
            plt.legend()
            plt.title('Accelerometer on Motion Thresholding') # subplot 211 title
            plt.savefig('accelerometer_motion_threshold.png') # TODO

        #Camera Plot

        if len(data['std']) > 0:
            m = np.array(data["mean"])
            max_ = np.array(data["max"])
            min_ = np.array(data["min"])
            s = np.array(data["std"])
            x = np.asarray(x_array)
            plt.xlim(0.1, x[-1]+1)
            yerr=[max_ - m, m - min_]
            plt.errorbar(x * 0.01, m.flatten(), yerr=s.flatten(), fmt='ok', lw=3)
            #plt.errorbar(x_array, m.flatten(), yerr=0.2, fmt='ok', lw=3)
            #plt.errorbar(x_array, m ,datayerr=yerr, fmt='.k', ecolor='gray', lw=1)
            plt.xlabel("Matching Threshold Variation")
            plt.ylabel("CUSUM Erro")
            plt.title('Camera on Motion Thresholding') # subplot 211 title
            plt.legend()
            plt.figure()
            plt.savefig('camera_motion_threshold.png') # TODO


        # data = np.array(userdata.data_in['odom'])
        # plt.figure()
        # plt.plot(x_array,data[:,0], label='Odometry Threshold X')
        # plt.plot(x_array,data[:,1], label='Odometry Threshold Y')
        # plt.plot(x_array,data[:,2], label='Odometry Threshold Z')
        # plt.xlabel("Window Size")
        # plt.ylabel("Maximum Threshold Detected")
        # plt.legend()
        # plt.title('Odometry Speeds on Motion Thresholding') # subplot 211 title
        # plt.savefig('odometry_motion_threshold.png') # TODO


        data = userdata.data_in['lidar']

        if len(data['std']) > 0:
            plt.figure() #TODO
            m = np.array(data["mean"])
            max_ = np.array(data["max"])
            min_ = np.array(data["min"])
            s = np.array(data["std"])
            x = np.asarray(x_array)
            plt.xlim(0.1, x[-1]+1)
            yerr=[max_ - m, m - min_]
            plt.errorbar(np.asarray(x_array), m.flatten(), yerr=s.flatten(), fmt='ok', lw=3)
            #plt.errorbar(np.asarray(x_array) * 0.01, np.array(data["mean"]), [np.array(data["mean"]) - np.array(data["min"]), np.array(data["max"]) - np.array(data["mean"])], fmt='.k', ecolor='gray', lw=1)
            plt.xlabel("Window Size")
            plt.ylabel("CUSUM Error")
            plt.legend()
            plt.title('Lidar on Motion Thresholding') # subplot 211 title
            plt.savefig('lidar_motion_threshold.png') # TODO


        data = userdata.data_in['mic']
        x_array = userdata.x_array

        if len(data['std']) > 0:
            plt.figure() #TODO
            m = np.array(data["mean"])
            max_ = np.array(data["max"])
            min_ = np.array(data["min"])
            s = np.array(data["std"])
            x = np.asarray(x_array)
            plt.xlim(0.1, x[-1]+1)

            yerr=[max_ - m, m - min_]
            plt.errorbar(np.asarray(x_array), m.flatten(), yerr=s.flatten(), fmt='ok', lw=3)
            #plt.errorbar(np.asarray(x_array) * 0.01, np.array(data["mean"]), [np.array(data["mean"]) - np.array(data["min"]), np.array(data["max"]) - np.array(data["mean"])], fmt='.k', ecolor='gray', lw=1)
            plt.xlabel("Window Size")
            plt.ylabel("CUSUM Error")
            plt.legend()
            plt.title('Microphone on Motion Thresholding') # subplot 211 title
            plt.savefig('mic_motion_threshold_sum.png') # TODO

        data = userdata.data_in['imu']
        x_array = userdata.x_array


        if len(data['std']) > 0:
            print (data["std"])
            for i in range(6):
                s = np.array(data["std"]).flatten().reshape(-1,6)[:,i]
                print (s.flatten())
                m = np.array(data["mean"]).flatten().reshape(-1,6)[:,i]
                _min = np.array(data["min"]).flatten().reshape(-1,6)[:,i]
                _max = np.array(data["max"]).flatten().reshape(-1,6)[:,i]
                x = np.asarray(x_array)
                plt.xlim(0.1, x[-1]+1)
                plt.figure() #TODO
                plt.errorbar(np.asarray(x_array), m.flatten(), yerr=s.flatten(), fmt='ok', lw=3)
                #plt.errorbar(np.asarray(x_array), m, m - _min, _max - m, fmt='.k', ecolor='gray', lw=1)
                plt.xlabel("Window Size")
                plt.ylabel("CUSUM sigma")
                plt.legend()
                plt.title('Imu on Motion Linear Accelerations Thresholding'+ str(i)) # subplot 211 title
                plt.savefig('imu_linear_motion_threshold' + str(i) + '.png') # TODO

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
                              input_keys=['acc_cum', 'cam_cum', 'odom_cum', 'lidar_cum', 'mic_cum', 'imu_cum', 'result_cum'],
                              output_keys=['acc_cum', 'cam_cum', 'odom_cum', 'lidar_cum', 'mic_cum', 'imu_cum ', 'result_cum'])
        self.finish_subscriber = rospy.Subscriber("/finish_reading", String, self.fb_cb)
        while self.finish_subscriber.get_num_connections() < 1:
                    print ("Wait finish_reading")
        rospy.sleep(2)
        self.current_counter = 0
        self.accel_thr = list()
        self.cam_thr = list()
        self.odom_thr = list()
        self.lidar_thr = list()
        self.mic_thr = list()
        self.imu_thr = list()
        self.stop_bag_request = False


        self.acc_cum = self.init_dict()
        self.cam_cum = self.init_dict()
        self.odom_cum = self.init_dict()
        self.lidar_cum = self.init_dict()
        self.mic_cum = self.init_dict()
        self.imu_cum = self.init_dict()

        for i in range(5):
            rospy.Subscriber("/collisions_"+str(i), sensorFusionMsg, self.threshold_cb, queue_size=100)

    def init_dict(self):
        dic = dict()
        dic["min"] = list()
        dic["max"] = list()
        dic["mean"] = list()
        dic["std"] = list()
        return dic


    def threshold_cb(self,msg):
        if msg.sensor_id.data == "accelerometer_1":
            self.accel_thr.append(msg.data)
        if msg.sensor_id.data == "cam1":
            self.cam_thr.append(msg.data)
        if msg.sensor_id.data == "odom_1":
            self.odom_thr.append(msg.data)
        if msg.sensor_id.data == "lidar_1":
            self.lidar_thr.append(msg.data)
        if msg.sensor_id.data == "mic_1":
            self.mic_thr.append(msg.data)
        if msg.sensor_id.data == "android_imu_1":
            self.imu_thr.append(msg.data)

        #print (msg.data)

    def fb_cb(self,msg):
        #print ("CB", msg)
        self.next_bag_request = True
        self.stop_bag_request = False

        if msg.data == "NEXT_BAG":
            self.current_counter = 0
        else:#FINISH
            #print ("current_counter" , self.current_counter)
            self.stop_bag_request = True
            #print ("accel_thr" , np.nanmax(self.accel_thr, axis=0) , " number of samples " , len(self.accel_thr)) if len(self.accel_thr)>0 else 0
            #print ("cam_thr" , np.nanmax(self.cam_thr, axis=0), " number of samples " , len(self.cam_thr)) if len(self.cam_thr)>0 else 0
            #print ("odom_thr" , np.nanmax(self.odom_thr, axis=0), " number of samples " , len(self.odom_thr)) if len(self.odom_thr)>0 else 0
            #print ("lidar_thr" , np.nanmax(self.lidar_thr, axis=0), " number of samples " , len(self.lidar_thr)) if len(self.lidar_thr)>0 else 0
            #print ("mic_thr" , np.nanmax(self.mic_thr, axis=0), " number of samples " , len(self.mic_thr)) if len(self.mic_thr)>0 else 0
            #print ("imu_thr" , np.nanmax(self.imu_thr, axis=0), " number of samples " , len(self.imu_thr)) if len(self.imu_thr)>0 else 0


    def execute(self, userdata):

        self.next_bag_request = False
        rospy.loginfo('Executing state MONITORING')
        #rospy.sleep(20)#TODO

        while not self.next_bag_request:
            pass
            #print ("waiting next_bag_request")

        rospy.sleep(0.2)
        self.next_bag_request = False

        if self.stop_bag_request:
            print ('Stop Received')
            if len(self.accel_thr) > 0:
                userdata.acc_cum["min"].append(np.nanmin(self.accel_thr, axis=0))
                userdata.acc_cum["max"].append(np.nanmax(self.accel_thr, axis=0))
                userdata.acc_cum["mean"].append(np.nanmean(self.accel_thr, axis =0))
                userdata.acc_cum["std"].append(np.nanstd(self.accel_thr, axis = 0))

            if len(self.cam_thr) > 0:
                userdata.cam_cum["min"].append(np.nanmin(self.cam_thr, axis=0))
                userdata.cam_cum["max"].append(np.nanmax(self.cam_thr, axis=0))
                userdata.cam_cum["mean"].append(np.nanmean(self.cam_thr, axis =0))
                userdata.cam_cum["std"].append(np.nanstd(self.cam_thr, axis = 0))

            if len(self.odom_thr) > 0:
                userdata.odom_cum["min"].append(np.nanmin(self.odom_thr, axis=0))
                userdata.odom_cum["max"].append(np.nanmax(self.odom_thr, axis=0))
                userdata.odom_cum["mean"].append(np.nanmean(self.odom_thr, axis =0))
                userdata.odom_cum["std"].append(np.nanstd(self.odom_thr, axis = 0))

            if len(self.lidar_thr) > 0:
                userdata.lidar_cum["min"].append(np.nanmin(self.lidar_thr, axis=0))
                userdata.lidar_cum["max"].append(np.nanmax(self.lidar_thr, axis=0))
                userdata.lidar_cum["mean"].append(np.nanmean(self.lidar_thr, axis =0))
                userdata.lidar_cum["std"].append(np.nanstd(self.lidar_thr, axis = 0))

            if len(self.mic_thr) > 0:
                userdata.mic_cum["min"].append(np.nanmin(self.mic_thr, axis=0))
                userdata.mic_cum["max"].append(np.nanmax(self.mic_thr, axis=0))
                userdata.mic_cum["mean"].append(np.nanmean(self.mic_thr, axis =0))
                userdata.mic_cum["std"].append(np.nanstd(self.mic_thr, axis = 0))

            if len(self.imu_thr) > 0:
                userdata.imu_cum["min"].append(np.nanmin(self.imu_thr, axis=0))
                userdata.imu_cum["max"].append(np.nanmax(self.imu_thr, axis=0))
                userdata.imu_cum["mean"].append(np.nanmean(self.imu_thr, axis =0))
                userdata.imu_cum["std"].append(np.nanstd(self.imu_thr, axis = 0))

            del self.accel_thr[:]
            del self.cam_thr[:]
            del self.odom_thr[:]
            del self.lidar_thr[:]
            del self.mic_thr[:]
            del self.imu_thr[:]
            print ("END_MONITOR")

            return 'END_MONITOR'
        else:
            print ("NEXT_MONITOR")
            return 'NEXT_MONITOR'


if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')
    start_sm("/home/jose/data/freemotion-1903/", "cob3-1903-", Monitor, Setup, Plotter, max_bag_file=210, max_window_size=75, start_window=2, step=3)
