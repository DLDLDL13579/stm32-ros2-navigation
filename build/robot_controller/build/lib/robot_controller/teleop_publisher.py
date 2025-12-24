#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

class TeleopPublisher(Node):
    def __init__(self):
        super().__init__('teleop_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print("ROS2 遥控发布器 - 按 'q' 退出")
        print("W: 前进, S: 后退, A: 左转, D: 右转, 空格: 停止")
        
        try:
            while rclpy.ok():
                key = self.get_key()
                twist = Twist()
                
                if key == 'w' or key == 'W':
                    twist.linear.x = 0.5
                    print("前进")
                elif key == 's' or key == 'S':
                    twist.linear.x = -0.5
                    print("后退")
                elif key == 'a' or key == 'A':
                    twist.angular.z = 1.0
                    print("左转")
                elif key == 'd' or key == 'D':
                    twist.angular.z = -1.0
                    print("右转")
                elif key == ' ':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    print("停止")
                elif key == 'q' or key == 'Q':
                    break
                else:
                    continue
                    
                self.publisher.publish(twist)
                
        except Exception as e:
            print(f"错误: {e}")
        finally:
            # 发布停止指令
            twist = Twist()
            self.publisher.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    node = TeleopPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    