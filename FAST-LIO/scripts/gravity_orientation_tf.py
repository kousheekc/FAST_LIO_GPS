#!/usr/bin/env python

"""
Suscribes to an Imu message with orientation to get the gravity information.
It then averages the first x gravity direction and finally publish the average gravity as a rotation tf between the odom frame and the world frame
"""

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3Stamped, Vector3
from sensor_msgs.msg import Imu
import math
import numpy as np


def normalize(q):
    """
    Normalizes a Quaternion message.

    :param q: The Quaternion message to normalize.
    :type q: Quaternion
    :return: The normalized Quaternion message.
    :rtype: Quaternion
    """
    if (q.x == 0 and q.x == q.y and q.y == q.z):
        q.w = 1.0
    else:
        q_norm = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
        q.x /= q_norm
        q.y /= q_norm
        q.z /= q_norm
        q.w /= q_norm
    return q


def imu_callback(imu_msg):
    global current_sample

    orientation = imu_msg.orientation
    orientation = normalize(orientation)

    if current_sample == num_samples:
        current_sample+=1
        # Define the number of iterations and the weighting factor
        num_iterations = 10
        alpha = 0.5

        # Initialize the mean quaternion to the first quaternion
        mean_quaternion = quaternions[0]

        # Iterate to compute the mean quaternion
        for i in range(num_iterations):
            # Initialize the weighted sum of quaternions
            weighted_sum = Quaternion()
            weighted_sum.x = 0.0
            weighted_sum.y = 0.0
            weighted_sum.z = 0.0
            weighted_sum.w = 0.0

            # Compute the weighted sum of quaternions
            for q in quaternions:
                dot_product = q.x*mean_quaternion.x + q.y*mean_quaternion.y + q.z*mean_quaternion.z + q.w*mean_quaternion.w
                if dot_product < -1.0:
                    dot_product = -1.0
                elif dot_product > 1.0:
                    dot_product = 1.0
                if dot_product < 0.0:
                    # Invert the quaternion to ensure the shortest path on the great circle arc
                    q.x *= -1.0
                    q.y *= -1.0
                    q.z *= -1.0
                    q.w *= -1.0
                    dot_product = -dot_product
                theta = np.arccos(dot_product)
                sin_theta = np.sin(theta)

                if sin_theta == 0.0:
                     # The quaternions are identical, so return one of them
                    w1 = 0.5
                    w2 = 0.5
                else:
                    w1 = np.sin((1.0 - alpha)*theta) / sin_theta
                    w2 = np.sin(alpha*theta) / sin_theta

                weighted_sum.x += w1*q.x + w2*mean_quaternion.x
                weighted_sum.y += w1*q.y + w2*mean_quaternion.y
                weighted_sum.z += w1*q.z + w2*mean_quaternion.z
                weighted_sum.w += w1*q.w + w2*mean_quaternion.w

            # Normalize the weighted sum to obtain the mean quaternion
            mean_quaternion = normalize(weighted_sum)

        # Compute transform from base_link to world frame
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()#imu_msg.header.stamp  last imu message time
        transform.header.frame_id = world_frame
        transform.child_frame_id = odom_frame
        transform.transform.rotation = mean_quaternion

        # Publish transform
        tf_br.sendTransform(transform)
        rospy.loginfo("Gravity upright world transform published")

    elif not (orientation.x == 0 and orientation.x == orientation.y and orientation.y == orientation.z):
        quaternions.append(orientation)
        current_sample+=1


if __name__ == '__main__':
    rospy.init_node('gravity_orientation')

    sub = rospy.Subscriber('input_imu_orientation', Imu, imu_callback)
    tf_br = tf2_ros.StaticTransformBroadcaster()

    odom_frame = rospy.get_param('/common/odom_frame', "odom")
    world_frame = rospy.get_param('world_frame', "world")

    num_samples = rospy.get_param('num_sample', 10)
    current_sample = 0
    quaternions = []

    rospy.spin()
