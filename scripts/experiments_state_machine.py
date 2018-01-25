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

# define state ReadBag
class MyBagReader(smach.State):
    def __init__(self):
        mytypes = [AccelStamped, Twist, Odometry, Odometry, LaserScan, LaserScan, LaserScan, Image, Image, Odometry]
        #self.path = '/home/jose/ROS/thesis_ws/my_ws/rosbag_test/cob3/static_runs_2911/static_runs/' #TODO
        self.path = '/home/jose/ROS/thesis_ws/my_ws/rosbag_test/cob3/cob-3-test-2001/'

        self.mytopics = ["/accel", "/cmd_vel", "/odom", "/base/odometry_controller/odom",
            "/scan_front", "/scan_rear", "/scan_unified",
            "/arm_cam3d/rgb/image_raw","/cam3d/rgb/image_raw", "/base/odometry_controller/odometry"]

        self.myPublishers = list()

        for topic_name, msg_type in zip(self.mytopics,mytypes):
            publisher = rospy.Publisher(topic_name, msg_type, queue_size=1)
            self.myPublishers.append([publisher,topic_name])

        smach.State.__init__(self,
                             outcomes=['RESTART_READER','END_READER'],
                             input_keys=['foo_counter_in', 'shared_string'],
                             output_keys=['foo_counter_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Reader')

        self.is_file_ok = False

        while not self.is_file_ok:
            try:
                file_name = self.path + userdata.shared_string + str(userdata.foo_counter_in)+".bag"
                self.bag = rosbag.Bag(file_name)
                self.is_file_ok = True
            except:
                userdata.foo_counter_out = userdata.foo_counter_in + 1
                print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")

        finish_pub = rospy.Publisher("finish_reading", String, queue_size=1)

        try:
            start_time = self.bag.get_start_time()
            end_time = self.bag.get_end_time()
            duration_time = end_time - start_time
            r = rospy.Rate(100) # Default 100
            for topic, msg, t in self.bag.read_messages(topics=self.mytopics):
                print ("ROSBag  Running ", t.to_sec() - start_time, " of " , duration_time, end="\r")
                for p, topic_name in self.myPublishers:
                    if topic_name == topic:
                        #print "printing on ", topic_name
                        p.publish(msg)
                        r.sleep()
                        break
            print ("\n")
            self.bag.close()
        except:
            print ("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb")
            self.bag.close()

        if userdata.foo_counter_in < 5:#110:  #n number of bag files // TODO default 35
            userdata.foo_counter_out = userdata.foo_counter_in + 1
            fb = String()
            fb.data = "NEXT_BAG"
            finish_pub.publish(fb)
            rospy.sleep(2)
            return 'RESTART_READER'
        else:
            fb = String()
            fb.data = "END_BAG"
            userdata.foo_counter_out = 1
            finish_pub.publish(fb)
            rospy.sleep(2)
            return 'END_READER'

class RestartReader(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['NEXT_BAG'],
                             input_keys=['bar_counter_in'])
        #rospy.spin()
        rospy.sleep(0.2)

    def execute(self, userdata):
        rospy.loginfo('Executing state RESTART READER')
        #rospy.loginfo('Counter = %f'%userdata.bar_counter_in)
        monitor_reset_pub = rospy.Publisher('/sm_reset', Empty, queue_size=1)
        while monitor_reset_pub.get_num_connections() < 1:
            pass
        #print (monitor_reset_pub.get_num_connections())
        monitor_reset_pub.publish(Empty())
        #print ("Send EMPTY")
        rospy.sleep(0.5)
        return 'NEXT_BAG'

class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['SETUP_DONE', 'FINISH'],
                             input_keys=['counter_in','acc_results', 'cam_results', 'odom_results', 'result_cum', 'results_', 'x_array'],
                             output_keys=['counter_out','acc_results', 'cam_results', 'odom_results', 'result_cum', 'results_', 'x_array'])
        #rospy.spin()
        self.acc_client = Client("accelerometer_process", timeout=3, config_callback=self.callback)
        self.cam_client = Client("vision_utils_ros", timeout=3, config_callback=self.callback)
        self.odom_client = Client("odom_collisions", timeout=3, config_callback=self.callback)

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
        self.cam_client.update_configuration({"mode": 1})
        rospy.sleep(0.5)
        if userdata.counter_in < 15:#0: # Define max TODO
            userdata.counter_out = userdata.counter_in + 5
            userdata.x_array.append(userdata.counter_in - 5)
            return 'SETUP_DONE'
        else:
            userdata.results_['accel'] = userdata.acc_results
            userdata.results_['cam'] = userdata.cam_results
            userdata.results_['odom'] = userdata.odom_results
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
        plt.plot(data[:,0], label='RGB Camera Thresholds')
        plt.xlabel("Run")
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
    def __init__(self, experiment_type):
        smach.State.__init__(self,
                             outcomes=['NEXT_MONITOR', 'END_MONITOR'],
                              input_keys=['acc_cum', 'cam_cum', 'odom_cum', 'result_cum'],
                              output_keys=['acc_cum', 'cam_cum', 'odom_cum', 'result_cum'])
        rospy.Subscriber("/finish_reading", String, self.fb_cb)
        self.current_counter = 0
        self.accel_thr = list()
        self.cam_thr = list()
        self.odom_thr = list()

        self.acc_cum = list()
        self.cam_cum = list()
        self.odom_cum = list()

        if experiment_type is "collisions_counter":
            for i in range(10):
                rospy.Subscriber("/collisions_"+str(i), sensorFusionMsg, self.counter_cb, queue_size=100)
        if experiment_type is "threshold_calculator":
            for i in range(10):
                rospy.Subscriber("/collisions_"+str(i), sensorFusionMsg, self.threshold_cb, queue_size=100)

    def threshold_cb(self,msg):
        if msg.sensor_id.data == "acc_1":
            self.accel_thr.append(msg.data)
        if msg.sensor_id.data == "cam_0":
            self.cam_thr.append(msg.data)
        if msg.sensor_id.data == "odom":
            self.odom_thr.append(msg.data)
        #print (msg.data)

    def counter_cb(self,msg):
        if msg.msg == 2:
            self.current_counter = self.current_counter + 1

    def fb_cb(self,msg):
        #print ("CB", msg)
        self.next_bag_request = True
        self.stop_bag_request = False

        if msg.data == "NEXT_BAG":
            self.current_counter = 0
        else:#FINISH
            #print ("current_counter" , self.current_counter)
            print ("accel_thr" , max(self.accel_thr) , " number of samples " , len(self.accel_thr))
            print ("cam_thr" , max(self.cam_thr), " number of samples " , len(self.cam_thr))
            print ("odom_thr" , max(self.odom_thr), " number of samples " , len(self.odom_thr))

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
            userdata.acc_cum.append(max(self.accel_thr))
            userdata.cam_cum.append(max(self.cam_thr))
            userdata.odom_cum.append(max(self.odom_thr))

            del self.accel_thr[:]
            del self.cam_thr[:]
            del self.odom_thr[:]
            return 'END_MONITOR'
        else:
            #print ("NEXT_MONITOR")
            return 'NEXT_MONITOR'

def monitor_cb(ud, msg):
    return None

def main():

    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['END_SM'])
    sm.userdata.window_size = 0
    #sm.userdata.bag_family = "cob3-attempt-2001-" #TODO
    sm.userdata.window_size_array = list()
    sm.userdata.results_ = dict()
    sm.userdata.acc_results = list()
    sm.userdata.cam_results = list()
    sm.userdata.odom_results = list()


    reading_sm = smach.StateMachine(outcomes=['END_READING_SM'])
    reading_sm.userdata.sm_counter = 1
    reading_sm.userdata.bag_family = "cob3-attempt-2001-" #TODO

    with reading_sm:
        smach.StateMachine.add('RESET_READING', RestartReader(),
                       transitions={'NEXT_BAG':'READING'},
                       remapping={'bar_counter_in':'sm_counter'})
        smach.StateMachine.add('READING', MyBagReader(),
                               transitions={'RESTART_READER':'RESET_READING',
                                            'END_READER':'END_READING_SM'},
                               remapping={'foo_counter_in':'sm_counter',
                                          'shared_string':'bag_family',
                                          'foo_counter_out':'sm_counter'})

    monitoring_sm = smach.StateMachine(outcomes=['END_MONITORING_SM'])
    monitoring_sm.userdata.results_ = sm.userdata.results_
    monitoring_sm.userdata.acc_results = sm.userdata.acc_results
    monitoring_sm.userdata.cam_results = sm.userdata.cam_results
    monitoring_sm.userdata.odom_results = sm.userdata.odom_results

    #montoring_sm.userdata.window_size_array = sm.window_size_array
    #experiment_type = "collisions_counter" #TODO
    experiment_type = "threshold_calculator" #TODO

    with monitoring_sm:
        smach.StateMachine.add('WAIT_FOR_READER', smach_ros.MonitorState("/sm_reset", Empty, monitor_cb),
                                transitions={'invalid':'MONITOR', 'valid':'WAIT_FOR_READER', 'preempted':'WAIT_FOR_READER'})
        smach.StateMachine.add('MONITOR', Monitor(experiment_type),
                       transitions={'NEXT_MONITOR':'WAIT_FOR_READER', 'END_MONITOR':'END_MONITORING_SM'},
                       remapping={'result_cum':'results_',
                                  'acc_cum':'acc_results',
                                  'cam_cum':'cam_results',
                                  'odom_cum': 'odom_results'})

    # Open the container
    with sm:
        smach.StateMachine.add('SETUP', Setup(),
                       transitions={'SETUP_DONE':'CON', 'FINISH': 'PLOT_RESULTS'},
                       remapping={'counter_in':'window_size',
                                  'counter_out':'window_size',
                                  'result_cum':'results_',
                                  'acc_cum':'acc_results',
                                  'cam_cum':'cam_results',
                                  'odom_cum':'odom_results',
                                  'x_array': 'window_size_array'})

        #Concurrent
        sm_con = smach.Concurrence(outcomes=['END_CON'],
                                   default_outcome='END_CON',
                                   outcome_map={#'RESTART':
                                       #{ 'MONITORING_SM':'RESTART_MONITOR',
                                         #'READ_SM' : 'RESTART_READER'},
                                         'END_CON':
                                         {'READ_SM': 'END_READING_SM',
                                          'MONITORING_SM': 'END_MONITORING_SM'}})
        # Open the container
        with sm_con:
            # Add states to the container
            #smach.Concurrence.add('WAIT_MONITOR', smach_ros.MonitorState("/sm_reset", Empty, monitor_cb))
            smach.Concurrence.add('READ_SM', reading_sm)
            smach.Concurrence.add('MONITORING_SM', monitoring_sm)

        #sm.userdata.data_in = monitoring_sm.userdata.acc_results #TODO
        sm.userdata.data_in = monitoring_sm.userdata.results_ #TODO

        smach.StateMachine.add('CON', sm_con,
                       transitions={#'RESTART':'CON',
                                    'END_CON':'SETUP'})
        smach.StateMachine.add('PLOT_RESULTS', Plotter(),
                       transitions={'PLOT_DONE':'END_SM'},
                       remapping={'data_in': 'data_in',
                                  'x_array': 'window_size_array'})

    # Execute SMACH plan
    #rospy.sleep(10)
    #outcome = sm.execute()
    #rospy.spin()

    #Instrospection
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    # Execute the state machine
    outcome = sm.execute()
    # Wait for ctrl-c to stop the application
    plt.show()
    sis.stop()

if __name__ == '__main__':
    main()
