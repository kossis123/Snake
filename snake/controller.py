#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class control(Node):
    def __init__(self):
        super().__init__("controller")
        self.pub=self.create_publisher(Float64MultiArray,"/cmd",10)
        self.create_timer(1,self.load)
        self.a=0.1
        self.omega=0.01
        self.phase=0.1
        self.t=0
    def load(self):
        self.thita=[0,0,0,0]
        msg=Float64MultiArray()
        for i in range(4):
            x=self.omega*self.t+i*self.phase
            self.thita[i]=self.a*math.sin(x)
            self.get_logger().info(str(self.thita[i]))
        self.t+=0.01
        msg.data=self.thita
        self.pub.publish(msg)
def main():
    rclpy.init()
    node=control()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__=="__main__":
    main()