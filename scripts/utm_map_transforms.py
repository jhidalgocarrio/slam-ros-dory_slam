#!/usr/bin/env python
import rospy
import math
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class Node():
    def __init__(self, name):
        self.name = name
        self.initialized = False
        self.init_odom = Odometry()

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header.stamp)

        map_odom_pub = rospy.Publisher("map/odom", Odometry, queue_size=1)

        if self.initialized is False:
            self.init_odom = data
            self.initialized = True
            rospy.loginfo(rospy.get_caller_id() + "\nInitial pose:\n %s", self.init_odom)

            # Get ROS static transformer
            static_br = tf2_ros.StaticTransformBroadcaster()
            static_t = geometry_msgs.msg.TransformStamped()

            # Publish the transformation
            static_t.header.stamp = self.init_odom.header.stamp
            static_t.header.frame_id = self.init_odom.header.frame_id
            static_t.child_frame_id = "map"
            static_t.transform.translation.x = self.init_odom.pose.pose.position.x
            static_t.transform.translation.y = self.init_odom.pose.pose.position.y
            static_t.transform.translation.z = self.init_odom.pose.pose.position.z
            static_t.transform.rotation = self.init_odom.pose.pose.orientation
            static_br.sendTransform(static_t)

        # Publish the topic for map odometry
        map_odom = Odometry()
        map_odom.header.stamp = data.header.stamp
        map_odom.header.frame_id = "map"
        map_odom.child_frame_id = data.child_frame_id
        map_odom.pose.pose.position.x = data.pose.pose.position.x - self.init_odom.pose.pose.position.x
        map_odom.pose.pose.position.y = data.pose.pose.position.y - self.init_odom.pose.pose.position.y
        map_odom.pose.pose.position.z = data.pose.pose.position.z - self.init_odom.pose.pose.position.z
        map_odom.pose.pose.orientation = data.pose.pose.orientation
        map_odom.pose.covariance = data.pose.covariance
        map_odom_pub.publish(map_odom)

        # Get ROS transformer Commented leaved it to C++ slam task
        #dynamic_br = tf2_ros.TransformBroadcaster()
        #dynamic_t = geometry_msgs.msg.TransformStamped()

        # Publish map to child_frame_id transform
        #dynamic_t.header.stamp = map_odom.header.stamp
        #dynamic_t.header.frame_id = map_odom.header.frame_id
        #dynamic_t.child_frame_id = map_odom.child_frame_id
        #dynamic_t.transform.translation.x = map_odom.pose.pose.position.x
        #dynamic_t.transform.translation.y = map_odom.pose.pose.position.y
        #dynamic_t.transform.translation.z = map_odom.pose.pose.position.z
        #dynamic_t.transform.rotation = map_odom.pose.pose.orientation

        #dynamic_br.sendTransform(dynamic_t)

    def utm_odometry_listener(self):

         # In ROS, nodes are uniquely named.
        rospy.init_node(self.name, anonymous=False)

        rospy.Subscriber("gps/odom", Odometry, self.callback)

         # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    node = Node("utm_map_transformer")
    node.utm_odometry_listener()


