#!usr/bin/python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Kill
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
import math
from functools import partial

class TurtleController(Node): 

    def __init__(self):
        super().__init__("turtle_controller")

        self.alive_turtles = []

        
        self.create_subscription(Turtle,"alive_turtles",self.callback_alive_turtles,10)
        self.create_subscription(Pose,"turtle1/pose",self.callback_pose,10)
        
        
        self.publisher_ = self.create_publisher(Twist,"turtle1/cmd_vel",10)
        self.get_logger().info("Turtle Controller Node has started")


    def callback_alive_turtles(self,msg):
        self.alive_turtles.append((msg.name,msg.x_coord, msg.y_coord))

    def callback_pose(self,pose):
        if self.alive_turtles:
            _,x,y = self.alive_turtles[0]
            x_dist = pose.x - x
            y_dist = pose.y - y
            move = Twist()

            dist = math.dist([x_dist],[y_dist])
            angular_distance = pose.theta - math.atan2(y_dist,x_dist)

            if dist > 0.03:
                move.linear.x = 6*dist
                
                if angular_distance > math.pi:
                    angular_distance -= 2*math.pi
                elif angular_distance < -math.pi:
                    angular_distance+= 2*math.pi
                
                move.angular.z = 6*angular_distance
            else:
                move.linear.x = 0.0
                move.linear.y = 0.0
                name,_,_ = self.alive_turtles.pop(0)

                self.call_kill_server(name)
            
            self.publisher_.publish(move)

    def call_kill_server(self,name):
        client = self.create_client(Kill,"kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting on Kill Server...")
        
        request = Kill.Request()
        request.name = name
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_kill_server,name=name))

    def callback_kill_server(self,future,name):
        try:
            response = future.result()
            self.get_logger().info(f"{name} has been killed")
        except Exception as e:
            self.get_logger().error("Error %r" %(e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()