

import sys


from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error, Logger
from rclpy.time import Time
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist


from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensors, kalmanFilter

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController


from geometry_msgs.msg import PoseStamped


from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class decision_maker(Node):
    
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, rate=10, motion_type=POINT_PLANNER):

        super().__init__("decision_maker")

        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos_profile=qos_publisher)


        self.create_subscription(PoseStamped, "/goal_pose", self.designPathFor, 10)
        
        self.pathPublisher = self.create_publisher(Path, '/designedPath', 10)

        # Trail of where the robot has actually been (EKF pose), shown live in rviz
        self.executedPathPublisher = self.create_publisher(Path, '/executedPath', 10)
        self.executedPath = []

        # Each goal gets its own pair of CSVs: path_<n>_planned.csv / path_<n>_executed.csv
        self.goalCount = 0
        self.executedLogger = None
        
        publishing_period=1/rate

        # TODO PART 5 choose your threshold
        self.reachThreshold= 0.01

        # TODO PART 5 your localization type
        self.localizer=localization(kalmanFilter)


        
        if motion_type==POINT_PLANNER:
            self.controller=controller(klp=0.8, klv=1.5, kap=0.8, kav=1.0)      
            self.planner=planner(POINT_PLANNER)


        
        elif motion_type==TRAJECTORY_PLANNER:
            # TODO PART 5 Put the gains that you concluded from lab 2
            self.controller=trajectoryController(klp=0.8, klv=1.5, kli=0.5, kap=0.8, kav=1.0, kai=0.8)      
            self.planner=planner(TRAJECTORY_PLANNER)
        
        else:
            print("Error! you don't have this type of planner", file=sys.stderr)


        self.goal = None

        self.create_timer(publishing_period, self.timerCallback)


        print("waiting for your input position, use 2D nav goal in rviz2")




    # This is for the rviz2 interface
    def designPathFor(self, msg: PoseStamped):
        
        spin_once(self.localizer)
        
        if self.localizer.getPose() is  None:
            print("waiting for odom msgs ....")
            return
        
        self.goal=self.planner.plan([self.localizer.getPose()[0], self.localizer.getPose()[1]],
                                     [msg.pose.position.x, msg.pose.position.y])

        if self.goal is None:
            print("no path found, choose another goal with 2D Goal Pose")
            return

        self.startLogging()

    def startLogging(self):

        self.goalCount += 1
        prefix = f"path_{self.goalCount}"

        # The A* output (or the single goal point for the point planner)
        path = self.goal if hasattr(self.goal[0], "__len__") else [self.goal]
        plannedLogger = Logger(f"{prefix}_planned.csv", ["x", "y"])
        for point in path:
            plannedLogger.log_values([point[0], point[1]])

        self.executedLogger = Logger(f"{prefix}_executed.csv",
                                     ["kf_x", "kf_y", "kf_th", "odom_x", "odom_y", "odom_th", "stamp"])

        # Start a fresh trail for this goal
        self.executedPath = []

        print(f"planned path saved to {prefix}_planned.csv, executed path logging to {prefix}_executed.csv")

    def logExecutedPose(self):

        pose = self.localizer.getPose()
        odomPose = self.localizer.getOdomPose()

        self.executedPath.append([pose[0], pose[1]])
        self.publishPathOnRviz2(self.executedPath, self.executedPathPublisher)

        if self.executedLogger is None or odomPose is None:
            return

        self.executedLogger.log_values([pose[0], pose[1], pose[2],
                                        odomPose[0], odomPose[1], odomPose[2],
                                        Time.from_msg(pose[3]).nanoseconds])

    
    def timerCallback(self):
        
        spin_once(self.localizer)

        if self.localizer.getPose() is  None:
            print("waiting for odom msgs ....")
            return
        
        
        vel_msg=Twist()
        
        if self.goal is None:
            return
        
        if type(self.goal) == list:
            reached_goal=True if calculate_linear_error(self.localizer.getPose(), self.goal[-1]) <self.reachThreshold else False
        else: 
            reached_goal=True if calculate_linear_error(self.localizer.getPose(), self.goal) <self.reachThreshold else False




        self.logExecutedPose()

        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()


            
            self.goal = None
            self.executedLogger = None
            print("waiting for the new position input, use 2D nav goal on map")

            return
        
        velocity, yaw_rate = self.controller.\
            vel_request(self.localizer.getPose(), self.goal, True)

        
        vel_msg.linear.x=velocity
        vel_msg.angular.z=yaw_rate
        
        self.publisher.publish(vel_msg)
        self.publishPathOnRviz2(self.goal)



    def publishPathOnRviz2(self, path, publisher=None):

        if publisher is None:
            publisher = self.pathPublisher

        Path_ =  Path()

        Path_.header.frame_id ="map"
        Path_.header.stamp = self.get_clock().now().to_msg()

        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]

            # Set the orientation of the pose. Here, it's set to a default orientation.
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            Path_.poses.append(pose)

        publisher.publish(Path_)

import argparse
def main(args=None):
    
    
    init()
    
    odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)
    
    if args.motion == "point":
        DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=POINT_PLANNER)
    elif args.motion == "trajectory":
        DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=TRAJECTORY_PLANNER)
    else:
        print("invalid motion type", file=sys.stderr)



    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")




if __name__=="__main__":
    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="trajectory")
    args = argParser.parse_args()

    main(args)
