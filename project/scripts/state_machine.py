import rospy
import math
import numpy as np
import rospy
import tf2_ros
from std_msgs.msg import String
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
import timeit
from std_msgs.msg import Bool


# callback to set the current pose from std_msgs.msg import String
def pose_callback(msg):
    global current_pose
    current_pose = msg

def localized_callback(msg):
    localized = msg.data


class StateMachine(object):
    def __init__(self):
        self.rotate_srv_nm = rospy.get_param(rospy.get_name() + '/rotate_srv')
        self.hover_srv_nm = rospy.get_param(rospy.get_name() + '/hover_srv')
        # Subscribe to topics
        sub_pose = rospy.Subscriber('cf1/pose', PoseStamped, pose_callback)
        sub_localize = rospy.Subscriber('is_initialized', Bool, localized_callback)

        # Wait for service providers
        rospy.wait_for_service(self.rotate_srv_nm, timeout=30)
        rospy.wait_for_service(self.hover_srv_nm)

        # Instantiate publishers
        self.pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)

        # Init state machine
        goal = None
        current_pose = None
        cmd = 0
        localized = False
        self.state = 0
        rospy.sleep(3)
        self.states()

    def states(self):

        # State 0: lift off and hover
        if self.state == 0:
            self.state = 1
            rospy.sleep(1)

        while not rospy.is_shutdown() and self.state != 4:

            # State 1: Generate next exploration goal from explorer
            if self.state == 1:

                self.state = 2
                rospy.sleep(1)

            # State 2: Generate path to next exploration goal and execute it
            if self.state == 2:

                self.state = 3
                rospy.sleep(1)

            # State 3: Rotate 90 degrees and hover a while three times while waiting for intruder detection
            if self.state == 3:

                self.state = 1
                rospy.sleep(1)

        # State 4: Land on the ground when explorer can't find more space to explore
        if self.state == 4:
            rospy.sleep(1)

        rospy.loginfo("%s: State machine finished!")
        return


if __name__ == '__main__':
    rospy.init_node('state_machine')
    try:
       StateMachine()
    except rospy.ROSInterruptException:
       pass

    rospy.spin()
