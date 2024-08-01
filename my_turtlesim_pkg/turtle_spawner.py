#!usr/bin/python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from my_robot_interfaces.msg import Turtle

from functools import partial
import random

class TurtleSpawner(Node): 

    def __init__(self):
        super().__init__("turtle_spawner")
        
        self.create_service(Kill, "kill_server",self.call_kill_server)
        self.publisher_ = self.create_publisher(Turtle,"alive_turtles",10)
        

        self.get_logger().info("Turtle Spawner Node has started.")
        self.create_timer(5,self.call_spawn_server)

    def call_spawn_server(self):
        x = round(random.uniform(0,11),2)
        y = round(random.uniform(0,11),2)
        theta = random.randint(1,360) * 0.01745
        
        client = self.create_client(Spawn,"spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting on Spawn Server...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn_server,x=x,y=y,theta=theta))

    def callback_spawn_server(self,future,x,y,theta):
        try:
            response = future.result()
            turtle = (response.name,x,y)
            self.get_logger().info(f"Turtle name: {turtle[0]} spawned at {turtle[1]},{turtle[2]}")
            msg = Turtle()
            msg.name, msg.x_coord, msg.y_coord = turtle
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error("Error %r" %(e,))

    def call_kill_server(self,name):
        client = self.create_client(Kill,"kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting on Kill Server...")
        
        request = Kill.Request()
        request.name = name
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_kill_server))

    def callback_kill_server(self,future):
        try:
            response = future.result()
            self.get_logger().info(f"Turtle {response.name} has been killed")
        except Exception as e:
            self.get_logger().error("Error %r" %(e,))




def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()