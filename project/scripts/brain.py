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

goal = None
current_pose = None
cmd = 0
state = 0
goal_nr = 0
localized = False

# This defines the new goal when the current goal is reached
def make_goal(goal, goal_nr):

    goal.header.frame_id = 'map'
    if goal_nr == 0:
        goal.pose.position.x += 2.0

        return goal

    if goal_nr == 1:
        goal.pose.position.y += 1.0

        return goal

    if goal_nr == 2:
        goal.pose.position.x -= 2.0

        return goal

    if goal_nr == 2:
        goal.pose.position. -= 1.0

        return goal

# Used to chech the distance from the current pose to the goal pose
def check_distance(current, goal, tol):
    t = 0

    dist = np.linalg.norm(current - goal)

    if dist < tol:
        t = 1

    return t

# Function to hover for a while
def hover(cmd):
    start = timeit.timeit()

    end = 0
    while end - start < 3:
        pub_cmd.publish(cmd)
        end += timeit.timeit()

def callback(msg):
    rospy.loginfo(msg)

# callback to set the current pose from std_msgs.msg import String
def pose_callback(msg):
    global current_pose
    current_pose = msg

def localized_callback(msg):
    localized = msg.data

# Transforms and publishes the goal cmd to cf1/cmd_position
def transform_goal(goal):
    global cmd
    # Need to tell TF that the goal was just generated
    goal.header.stamp = rospy.Time(0)

    if not tf_buf.can_transform(goal.header.frame_id, 'cf1/odom', goal.header.stamp):
        rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % goal.header.frame_id)
        return

    # Transform setpoint from map to cf1/odom
    goal_odom = tf_buf.transform(goal, 'cf1/odom')

    cmd = Position()

    cmd.header.stamp = goal.header.stamp
    cmd.header.frame_id = goal_odom.header.frame_id

    cmd.x = goal_odom.pose.position.x
    cmd.y = goal_odom.pose.position.y
    cmd.z = goal_odom.pose.position.z

    roll, pitch, yaw = euler_from_quaternion((goal_odom.pose.orientation.x,
                                              goal_odom.pose.orientation.y,
                                              goal_odom.pose.orientation.z,
                                              goal_odom.pose.orientation.w))
    cmd.yaw = math.degrees(yaw)

    return cmd
    pub_cmd.publish(cmd)

# Initiate node
rospy.init_node('brain')

# Initiate TF buffer and listener
tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)

# Initiate subscribers and publisher
sub_pose = rospy.Subscriber('cf1/pose', PoseStamped, pose_callback)
localized_sub = rospy.Publisher('is_initialized', Bool, localized_callback)
pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=20)

# Sleep to allow queues to fill up
rospy.sleep(3)

def main():

    global current_pose, goal, cmd, localized
    rate = rospy.Rate(10)  # Hz
    tol = 0.1

    while not rospy.is_shutdown():

        if state == 0:

            goal_msg = current_pose

            goal_msg.z += 0.5

            Hover(goal_msg)

            state = 1

        if state == 1:

            while not localized:
                Hover(goal_msg)

            state = 2


        if state == 2:

            goal_msg = current_pose

            if goal_nr > 3:
                goal_nr = 0

            else:

                goal_msg = make_goal(goal_msg, goal_nr)

                goal_position = np.array([goal_msg.pose.position.x, goal_msg.pose.position.y])
                current_position = np.array([current_pose.pose.position.x, current_pose.pose.position.y])

                while not check_distance(current_position, goal_position, tol):

                    transformed_goal =transform_goal(goal_msg)
                    pub_cmd.publish(transformed_goal)

                Hover(transformed_goal)

                goal_nr += 1

            state = 1

        rate.sleep()

if __name__ == '__main__':
    main()


# state = 0
# drone_localized = False

# while rospy.isnot(shutdown):

#    if state == 0:
        # take off and hover
#        state == 1

#    if state == 1:
        # localize
#        drone_localized = True
#        state = 2

#    if state == 2:
        # plan path from file (milestone 2)
        # execute path
#        if drone_localized == False:
#            state = 1
#        else:
#            continue
