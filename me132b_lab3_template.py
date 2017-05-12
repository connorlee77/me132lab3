#!/usr/bin/env python

# ME132B - Lab 3     Estimate the position of the robot using the lidar scans and
# Extra credit       wheel odometry for initial estimates

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState #must use 'sudo apt-get install ros-indigo-kobuki-msgs' first
from sensor_msgs.msg import LaserScan
import time
import math
import tf

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
        self.previous_scan = 0
        self.prev_left_encoder = 0
        self.prev_right_encoder = 0
        
        # After calculating your wheel odometry rates you'll want to save those for the lidar processing
        self.del_x = 0
        self.del_y = 0
        self.del_theta = 0
        
        # These incremental changes can be added to your current estimate of the robot's position
        self.curr_x = 0
        self.curr_y = 0
        self.curr_theta = 0
        
        # Wait until we receive messages to process in the callback functions
        rospy.spin()
    
    # Read in and process data from the encoders, this is called whenever encoder messages come in
    def processEncoder(self,data):
        # Do your processing here, in the same way as the previous lab
        self.prev_left_encoder = data.left_encoder
        self.prev_right_encoder = data.right_encoder
    
    # Read in and process data from the scan topic, this is called whenever scan messages come in
    def processLidar(self,data):
        # Do your processing here, you can find info on the 'laserscan' message at docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
        self.previous_scan = data # Maybe it would be better to save the scan once it's processed into coordinates?
        
        # Once you have an estimate of the change in your position, update your current estimate and broadcast it
        self.br.sendTransform((self.curr_x,self.curr_y,0.0),tf.transformations.quaternion_from_euler(0, 0, self.curr_theta),rospy.Time.now(),"base_link","world")

    # Unsubscribe from the sensor topic and tell the turtlebot to stop moving
    def shutdown(self):
        self.encoder_subscriber.unregister()
        self.lidar_subscriber.unregister()
        rospy.loginfo("Node stopped")
 
if __name__ == '__main__':
    LidarOdometry()

