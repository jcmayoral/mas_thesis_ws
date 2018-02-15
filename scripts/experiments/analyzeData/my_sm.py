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
from common_states import MyBagReader, RestartReader

def monitor_cb(ud, msg):
    return None

# Create a SMACH state machine
def start_sm(path, common_string, monitor_state, setup_state, plot_state, time_limit = float("inf")):
  sm = smach.StateMachine(outcomes=['END_SM'])
  sm.userdata.window_size = 2
  #sm.userdata.bag_family = "cob3-attempt-2001-" #TODO
  sm.userdata.window_size_array = list()
  sm.userdata.results_ = dict()
  sm.userdata.acc_results = list()
  sm.userdata.cam_results = list()
  sm.userdata.odom_results = list()


  reading_sm = smach.StateMachine(outcomes=['END_READING_SM'])
  reading_sm.userdata.sm_counter = 1
  reading_sm.userdata.path = path
  reading_sm.userdata.bag_family = common_string #TODO

  with reading_sm:
      smach.StateMachine.add('RESET_READING', RestartReader(),
                     transitions={'NEXT_BAG':'READING'},
                     remapping={'bar_counter_in':'sm_counter'})
      smach.StateMachine.add('READING', MyBagReader(limit = time_limit),
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

  with monitoring_sm:
      smach.StateMachine.add('WAIT_FOR_READER', smach_ros.MonitorState("/sm_reset", Empty, monitor_cb),
                              transitions={'invalid':'MONITOR', 'valid':'WAIT_FOR_READER', 'preempted':'WAIT_FOR_READER'})
      smach.StateMachine.add('MONITOR', monitor_state(),
                     transitions={'NEXT_MONITOR':'WAIT_FOR_READER', 'END_MONITOR':'END_MONITORING_SM'},
                     remapping={'result_cum':'results_',
                                'acc_cum':'acc_results',
                                'cam_cum':'cam_results',
                                'odom_cum': 'odom_results',
                                'imu_cum': 'imu_results'})

  # Open the container
  with sm:
      smach.StateMachine.add('SETUP', setup_state(),
                     transitions={'SETUP_DONE':'CON', 'FINISH': 'PLOT_RESULTS'},
                     remapping={'counter_in':'window_size',
                                'counter_out':'window_size',
                                'result_cum':'results_',
                                'acc_cum':'acc_results',
                                'cam_cum':'cam_results',
                                'odom_cum':'odom_results',
                                'imu_cum': 'imu_results',
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
      smach.StateMachine.add('PLOT_RESULTS', plot_state(),
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
