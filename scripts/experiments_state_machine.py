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
from geometry_msgs.msg import AccelStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan

# define state ReadBag
class MyBagReader(smach.State):
    def __init__(self):
        mytypes = [AccelStamped, Twist, Odometry, Odometry, LaserScan, LaserScan, LaserScan, Image, Image]
        self.path = '/home/jose/ROS/thesis_ws/my_ws/rosbag_test/cob3/static_runs_2911/static_runs/' #TODO
        self.mytopics = ["/accel", "/cmd_vel", "/odom", "/base/odometry_controller/odom",
            "/scan_front", "/scan_rear", "/scan_unified",
            "/arm_cam3d/rgb/image_raw","/cam3d/rgb/image_raw"]

        self.myPublishers = list()

        for topic_name, msg_type in zip(self.mytopics,mytypes):
            publisher = rospy.Publisher(topic_name, msg_type, queue_size=1)
            self.myPublishers.append([publisher,topic_name])

        smach.State.__init__(self,
                             outcomes=['outcome1','outcome2'],
                             input_keys=['foo_counter_in', 'shared_string'],
                             output_keys=['foo_counter_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')

        file_name = self.path + userdata.shared_string + str(userdata.foo_counter_in)+".bag"
        self.bag = rosbag.Bag(file_name)

        start_time = self.bag.get_start_time()
        end_time = self.bag.get_end_time()

        duration_time = end_time - start_time

        r = rospy.Rate(100)

        for topic, msg, t in self.bag.read_messages(topics=self.mytopics):
            print ("ROSBag  Running ", t.to_sec() - start_time, " of " , duration_time, end="\r")
            for p, topic_name in self.myPublishers:
                if topic_name == topic:
                    #print "printing on ", topic_name
                    p.publish(msg)
                    r.sleep()
                    break
        self.bag.close()

        if userdata.foo_counter_in < 2:
            userdata.foo_counter_out = userdata.foo_counter_in + 1
            return 'outcome1'
        else:
            return 'outcome2'

# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1'],
                             input_keys=['bar_counter_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.loginfo('Counter = %f'%userdata.bar_counter_in)
        return 'outcome1'


def main():

    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter = 1
    sm.userdata.bag_family = "static" #TODO

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('READ', MyBagReader(),
                               transitions={'outcome1':'BAR',
                                            'outcome2':'outcome4'},
                               remapping={'foo_counter_in':'sm_counter',
                                          'shared_string':'bag_family',
                                          'foo_counter_out':'sm_counter'})
        smach.StateMachine.add('BAR', Bar(),
                               transitions={'outcome1':'READ'},
                               remapping={'bar_counter_in':'sm_counter'})


    # Execute SMACH plan
    #outcome = sm.execute()

    #Instrospection
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    # Execute the state machine
    outcome = sm.execute()
    # Wait for ctrl-c to stop the application
    #rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
