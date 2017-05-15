#!/usr/bin/env python

# ME132B - Lab 3     Estimate the position of the robot using the lidar scans and
# Extra credit       wheel odometry for initial estimates

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState #must use 'sudo apt-get install ros-indigo-kobuki-msgs' first
from sensor_msgs.msg import LaserScan
import numpy as np 
import time
import math
import tf
import matplotlib.pyplot as plt
import heapq

class LidarOdometry():
    def __init__(self):
        # initiliaze
        rospy.init_node('ME132b_Lab_3', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop the node hit CTRL + C")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
        # Subscribe to the sensors/core topic to read the encoder values, and scan to read lidar data
        self.encoder_subscriber = rospy.Subscriber("/mobile_base/sensors/core",SensorState,self.processEncoder)
        self.lidar_subscriber = rospy.Subscriber("/scan",LaserScan,self.processLidar)
        
        # Create a tf broadcaster so that we can publish the position that we calculate
        self.br = tf.TransformBroadcaster()
        
        # You'll probably want to save the previous scan and encoder values for comparison with the next ones
        self.previous_scan = None
        self.prev_left_encoder = 0
        self.prev_right_encoder = 0
        
        # After calculating your wheel odometry rates you'll want to save those for the lidar processing
        self.del_x = 0
        self.del_y = 0
        self.del_theta = 0
        
        # These incremental changes can be added to your current estimate of the robot's position
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_theta = 0.0

        # Wait until we receive messages to process in the callback functions
        rospy.spin()
    
    # Read in and process data from the encoders, this is called whenever encoder messages come in
    def processEncoder(self,data):
        # Do your processing here, in the same way as the previous lab
        W = 0.23 / 2
        rho = 0.035

        right_enc = data.right_encoder*2.0*np.pi/2578.33
        left_enc = data.left_encoder*2.0*np.pi/2578.33
        dphi1 = right_enc - self.prev_right_encoder
        dphi2 = left_enc - self.prev_left_encoder
        theta = self.curr_theta

        
        self.del_x = (dphi2 + dphi1) * np.cos(theta)
        self.del_y = (dphi2 + dphi1) * np.sin(theta)
        self.del_theta = -(dphi2 - dphi1)/W 

        self.curr_x += (rho / 2) * self.del_x
        self.curr_y += (rho / 2) * self.del_y
        self.curr_theta += (rho / 2) * self.del_theta

        self.prev_left_encoder = left_enc
        self.prev_right_encoder = right_enc

    # Read in and process data from the scan topic, this is called whenever scan messages come in
    def processLidar(self,data):
        # Do your processing here, you can find info on the 'laserscan' message at docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
        
        ### Set ICP parameters here
        M = 30
        threshold = 10

        
        dx = self.del_x
        dy = self.del_y
        dTheta = self.del_theta

        # Get scan data
        angles = np.arange(data.angle_min, data.angle_max + data.angle_increment, data.angle_increment)
        ranges = data.ranges

        # Retrieve current coordinates from scan and previous coordinates.
        curr_coordinates = np.array([ranges * np.cos(angles), ranges * np.sin(angles)])
        prev_coordinates = self.previous_scan

        # Skip if no previous scans exist
        if self.previous_scan == None:
            self.previous_scan = curr_coordinates
            self.br.sendTransform((self.curr_x,self.curr_y,0.0),tf.transformations.quaternion_from_euler(0, 0, self.curr_theta),rospy.Time.now(),"base_link","world")
            return

        # Initialize starting values
        translation = np.array([[dx], [dy]]) 
        rotation =  np.array([[np.cos(dTheta), -np.sin(dTheta)], [np.sin(dTheta), np.cos(dTheta)]])
        # ICP iterations
        error = 1000000000
        while error > threshold:
            
            # Transform points in V_{k+1} to the coordinate system in V_{k}
            displaced_curr_coordinates = rotation.dot(curr_coordinates) + np.array([translation]).T

            h = []
            # Get M matching points via heapsort
            for pt1 in displaced_curr_coordinates.T:
                for pt2 in prev_coordinates.T:
                    d = np.linalg.norm(pt1 - pt2)
                    h.append([d, pt1, pt2])

            M_smallest = heapq.nsmallest(M, h, key=lambda x: x[0])

            # Organize the M matching points
            i = 0
            curr = np.zeros((M, 2))
            prev = np.zeros((M, 2))
            for _, pt1, pt2 in M_smallest:
                curr[i] = np.squeeze(pt1)
                prev[i] = np.squeeze(pt2)

                i += 1

            # Calculate translation from M matching points
            prev_translation = np.copy(translation)
            translation = np.mean(prev, axis=0).T - rotation.dot(np.mean(curr, axis=0).T)

            dx, dy = np.squeeze(translation)
            x, y = np.squeeze(np.mean(prev, axis=0).T)
            xp, yp = np.squeeze(np.mean(curr, axis=0).T)

            # Calculate rotation from M matching points
            dTheta = np.arctan((y*dx - x*dy + x*yp -y*xp) / (y*dy + x*dx -(y*yp+x*xp)))
            rotation = np.array([[np.cos(dTheta), -np.sin(dTheta)], [np.sin(dTheta), np.cos(dTheta)]])

            # Calculate error
            error = np.sum(np.square(np.linalg.norm(rotation.dot(curr.T) + np.array([translation]).T - prev.T, axis=0)))

        # Update the current pose
        self.curr_theta += dTheta
        self.curr_x, self.curr_y = np.squeeze(rotation.dot(np.array([[self.curr_x], [self.curr_y]])) + np.array([translation]).T)

        self.previous_scan = curr_coordinates # Maybe it would be better to save the scan once it's processed into coordinates?
        
        # Once you have an estimate of the change in your position, update your current estimate and broadcast it
        self.br.sendTransform((self.curr_x,self.curr_y,0.0),tf.transformations.quaternion_from_euler(0, 0, self.curr_theta),rospy.Time.now(),"base_link","world")

    # Unsubscribe from the sensor topic and tell the turtlebot to stop moving
    def shutdown(self):
        self.encoder_subscriber.unregister()
        self.lidar_subscriber.unregister()
        rospy.loginfo("Node stopped")
 
if __name__ == '__main__':
    LidarOdometry()

