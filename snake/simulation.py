import rclpy
from rclpy.node import Node
import pybullet as p
import pybullet_data
from std_msgs.msg import Float64MultiArray
class sim(Node):
    def __init__(self):
        super().__init__("node2")
        self.sub=self.create_subscription(Float64MultiArray,"/cmd",self.callback,10)
        self.create_timer(0.1,self.load)
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane=p.loadURDF('plane.urdf')
        self.robot=p.loadURDF('/home/badassatron/ros2_ws/src/snake/urdf/snake.urdf',basePosition=[0,0,-0.15],useFixedBase=False)
        p.setGravity(0,0,-9.8)
        self.number=p.getNumJoints(self.robot)
        self.list=[0.0]*self.number
        for i in range(-1,self.number):
            p.changeDynamics(self.robot,i,lateralFriction=2.0)
        p.changeDynamics(self.plane, -1, lateralFriction=2.0)

    def callback(self,x:Float64MultiArray):
        for i in range(self.number):
            self.list[i]=x.data[i]
            
    def load(self):
        for i, target in enumerate(self.list):
            p.setJointMotorControl2(
                self.robot,
                i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target

            )
        p.stepSimulation()

def main():
    rclpy.init()
    node=sim()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()