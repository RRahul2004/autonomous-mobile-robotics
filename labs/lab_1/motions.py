# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile

# TODO Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

# You may add any other imports you may need/want to use below
# import ...

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius_=0.0
        self.spiral_rate =0.0
        self.spiral_rate_max = 1.0
        self.reached_spiral_max = False
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        self.vel_publisher=self.create_publisher(Twist, "cmd_vel", 10)
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        # Looked up typical settings for QoS profile
        qos=QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT ,
            durability = DurabilityPolicy.VOLATILE,
            history = HistoryPolicy.KEEP_LAST,
            depth = 10
        )

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # IMU subscription
        
        self.imu_subscription = self.create_subscription(Imu, "imu", self.imu_callback, qos)
        
        # ENCODER subscription

        self.enc_subscription = self.create_subscription(Odometry, "odom", self.odom_callback, qos)
        
        # LaserScan subscription 
        
        self.lidar_subscription = self.create_subscription(LaserScan, "scan", self.laser_callback, qos)
        
        self.create_timer(0.1, self.timer_callback)


    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        # log imu msgs
        #obatined acc_x, acc_, angular_z, timestamp from imu_msg
        acc_x = imu_msg.linear_acceleration.x
        acc_y = imu_msg.linear_acceleration.y
        angular_z = imu_msg.angular_velocity.z
        timestamp = Time.from_msg(imu_msg.header.stamp).nanoseconds

        # log values to the csv (append)
        self.imu_logger.log_values([acc_x, acc_y, angular_z, timestamp])

        #set flag to true
        self.imu_initialized = True


        
        
    def odom_callback(self, odom_msg: Odometry):
        
        # log odom msgs
        #obtained x and y position data
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        
        # obtained orientation in quaternion 
        orientation = odom_msg.pose.pose.orientation
        quat =  [orientation.x, orientation.y, orientation.z, orientation.w]
        
        # converted quat to euler and only return yaw
        th = euler_from_quaternion(quat)

        #returned timestamp
        timestamp = Time.from_msg(odom_msg.header.stamp).nanoseconds

        # log values to the csv (append)
        self.odom_logger.log_values([x, y, th, timestamp])

        #set flag to true
        self.odom_initialized = True
                
    def laser_callback(self, laser_msg: LaserScan):
        
        # log laser msgs with position msg at that time
        # obtained ranges array, angle increment, and timestamnp
        ranges = laser_msg.ranges
        angle_increment = laser_msg.angle_increment
        timestamp = Time.from_msg(laser_msg.header.stamp).nanoseconds

        # log values to csv (append)
        self.laser_logger.log_values([ranges,angle_increment, timestamp])

        #set flag to true
        self.laser_initialized = True

    def timer_callback(self):
        
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self):
        
        msg=Twist()
        # fill up the twist msg for circular motion
        #for circular motion mag(lin vel) = mag(ang vel)
        msg.linear.x = 0.5
        msg.linear.y = 0.0
        msg.angular.z = 1.0
        return msg

    def make_spiral_twist(self):
        msg=Twist()
        # fill up the twist msg for spiral motion

        # keep angualr velocity constant and then slowly increment linear velocity
        msg.linear.x = self.spiral_rate
        msg.linear.y = 0.0
        msg.angular.z = 1.0

        if self.reached_spiral_max == False:
            self.spiral_rate += 0.01
            if self.spiral_rate >= self.spiral_rate_max:
                self.reached_spiral_max = True
        if self.reached_spiral_max == True:
            self.spiral_rate -= 0.01
            if self.spiral_rate <= 0:
                self.reached_spiral_max = False
        
        
        
        

        return msg
    
    def make_acc_line_twist(self):
        msg=Twist()
        # fill up the twist msg for line motion
        #have non-zero lin velocity and ang vel = 0
        msg.linear.x = 0.5
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        return msg

import argparse

if __name__=="__main__":
    

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")



    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {args.motion.lower()} motion type")


    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
