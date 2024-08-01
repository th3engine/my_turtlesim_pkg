#!usr/bin/python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
import math

class TurtleController(Node): 

    def __init__(self):
        super().__init__("turtle_controller")

        self.alive_turtles = []

        self.coordinates = (0,0)
        self.pose = Pose()
        self.create_subscription(Turtle,"alive_turtles",self.callback_alive_turtles,10)
        self.create_subscription(Pose,"turtle1/pose",self.callback_pose,10)
        
        
        self.publisher_ = self.create_publisher(Twist,"turtle1/cmd_vel",10)
        self.get_logger().info("Turtle Controller Node has started")


    def callback_alive_turtles(self,msg):
        self.alive_turtles.append((msg.name,msg.x_coord, msg.y_coord))

    def callback_pose(self,msg):
        if self.alive_turtles:
            _,x,y = self.alive_turtles[0]
            x_dist = msg.x - x
            y_dist = msg.y - y
            move = Twist()

            dist = math.dist([x_dist],[y_dist])
            angle = math.atan(y_dist/x_dist)

            angular_distance = msg.theta - angle

            
            

            move.angular.x = 0.0
            move.angular.y = 0.0
            
            move.linear.x = 0.0
            move.linear.y = 0.0
            move.linear.z = 0.0

            current_dist = 0
            t0,_ = self.get_clock().now().seconds_nanoseconds()

            angular_velocity = angular_distance/5

            while int(round(angular_velocity,4)) != 0:
                move.angular.z = angular_velocity
                self.publisher_.publish(move)
            move.angular.z = 0.0

        
            
            while current_dist<dist:
                self.publisher_.publish(move)
                t1,_ = self.get_clock().now().seconds_nanoseconds()
                current_dist = math.sqrt(0.1**2+0.1**2) * (t1-t0)
                move.linear.x = 0.1
                move.linear.y = 0.1
            move.linear.x = 0.0
            move.linear.y= 0.0


                

            



    

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()