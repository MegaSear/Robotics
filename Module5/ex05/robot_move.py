import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi

class CirclePublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 1)
        self.timer_period = 0.25
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        twist = Twist()
        twist.linear.x = float(8)

        N1 = int(4.0/self.timer_period)
        N2 = int(6.5/self.timer_period)
        N3 = int(7.5/self.timer_period)

        if self.count in list(range(N1, N2)):
            twist.angular.z = pi
            twist.linear.y = float(0.3)

        if self.count in list(range(N2, N3)):
            twist.angular.z = -pi/2

        if self.count >= N3:
            self.count = 0
            
        self.count += 1  
        self.publisher.publish(twist)    
        
def main(args=None):
    rclpy.init(args=args)
    circling = CirclePublisher()
    rclpy.spin(circling)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
