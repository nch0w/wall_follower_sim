#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    WALL_TOPIC = "/wall"
    prev_error = None

    def __init__(self):
        # Initialize your publishers and
        # subscribers here
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.scan_sub = rospy.Subscriber(self.SCAN_TOPIC, numpy_msg(LaserScan), self.scan_callback)
        self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=1)

        # # drive forward in a straight line
        # self.drive_msg = AckermannDriveStamped()
        # self.drive_msg.drive.speed = self.VELOCITY
        # self.drive_msg.drive.steering_angle = 0.0


    # TODO:
    # Write your callback functions here.
    def scan_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        
        # convert scan data to x, y coordinates
        range_angles = np.linspace(angle_min, angle_max, len(ranges))
        range_vectors = np.vstack([ranges * np.cos(range_angles), ranges * np.sin(range_angles)]).T

        # self.SIDE = 1

        # mask out the ranges we care about
        if self.SIDE == 1:
            mask = (range_angles > (-np.pi/3)) & (range_angles < (np.pi/2)) & (ranges < 4)
        else:
            mask = (range_angles < (np.pi/3)) & (range_angles > -(np.pi/2))  & (ranges < 4)

        if np.sum(mask) == 0:
            if self.SIDE == 1:
                mask = (range_angles > (-np.pi/3)) & (range_angles < (np.pi/2)) 
            else:
                mask = (range_angles < (np.pi/3)) & (range_angles > -(np.pi/2))
        
        # if we're too close, just go forward
        if np.min(ranges) < 0.2:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = self.VELOCITY
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)
            return

        range_vectors = range_vectors[mask]

        xs = np.vstack((range_vectors[:, 0], np.ones(range_vectors.shape[0]))).T
        ys = range_vectors[:, [1]]

        xs = xs.astype(float)
        ys = ys.astype(float)

        m, c = np.linalg.lstsq(xs, ys)[0]
        m = m[0]
        c = c[0]

        # VisualizationTools.plot_line([c, -2], [c, 2], self.line_pub, frame="/laser")
        # VisualizationTools.plot_line([-2, 2], [c-1, c+1], self.line_pub, frame="/laser")
        # VisualizationTools.plot_line([-2, 2], [c-2*m, c+2*m], self.line_pub, frame="/laser")
        
        x_mid = (-c * m) / (m**2 + 1)
        y_mid = m * x_mid + c

        # for i in range(range_vectors.shape[0]):
        #     x = [0, range_vectors[i][0]]
        #     y = [0, range_vectors[i][1]]
        #     VisualizationTools.plot_line(x, y, self.line_pub, frame="/laser")

        VisualizationTools.plot_line([0, x_mid], [0, y_mid], self.line_pub, frame="/laser")
        actual_dist = np.sqrt(x_mid**2 + y_mid**2)

        dist_error = actual_dist - self.DESIRED_DISTANCE

        # if dist error is too high, turn hard to get back
        if dist_error > 1:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = self.VELOCITY
            drive_msg.drive.steering_angle = 0.34 * self.SIDE
            self.drive_pub.publish(drive_msg)
            return

        if self.prev_error is None:
            self.prev_error = dist_error
        
        # rospy.loginfo("actual_dist: " + str(actual_dist))
        # rospy.loginfo("desired dist: " + str(self.DESIRED_DISTANCE))
        # rospy.loginfo("dist_error: " + str(dist_error))

        drive_msg = AckermannDriveStamped()
        # drive_msg.drive.speed = 0
        # drive_msg.drive.steering_angle = 0.0

        # PID controller to make robot reach desired distance from wall
        kP = 4
        kD = 1
        kP_angle = 1

        angle_error = -np.arctan2(m, 1)
        rospy.loginfo("angle_error: " + str(angle_error))

        # drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = np.clip(-kP_angle * angle_error + (-kP) * dist_error * (-self.SIDE) + kD * (dist_error - self.prev_error), -0.34, 0.34)
        drive_msg.drive.speed = self.VELOCITY

        # rospy.loginfo("steering angle: " + str(drive_msg.drive.steering_angle * (-self.SIDE)))
        # rospy.loginfo("prop update: " + str(-kP * dist_error))
        # rospy.loginfo("derivative update: " + str(kD * (dist_error - self.prev_error)))

        self.prev_error = dist_error    
        # drive_msg.drive.steering_angle = 0
        # drive_msg.drive.speed = 0

        # publish the drive message
        self.drive_pub.publish(drive_msg)


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
