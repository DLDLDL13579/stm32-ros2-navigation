#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios
import threading

class RobotControllerSTM32(Node):
    def __init__(self):
        super().__init__('robot_controller_stm32')
        
        # 发布者，用于发送控制指令到STM32接口
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 控制参数
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.key_linear = 0.0
        self.key_angular = 0.0
        
        # 键盘监听控制
        self.keyboard_active = True
        self.settings = termios.tcgetattr(sys.stdin)
        
        # 创建定时器发布控制指令
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.get_logger().info("STM32机器人控制器已启动")
        
    def get_key(self):
        """获取键盘输入（非阻塞）"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def keyboard_listener(self):
        """键盘监听线程"""
        print("\n=== STM32小车控制说明 ===")
        print("W: 前进, S: 后退, A: 左转, D: 右转")
        print("空格: 停止, Q: 退出程序")
        print("1-9: 调整速度等级 (1最慢, 9最快)")
        print("===================\n")
        
        while self.keyboard_active and rclpy.ok():
            key = self.get_key()
            
            if key == 'w' or key == 'W':
                self.key_linear = self.linear_speed
                print("前进")
            elif key == 's' or key == 'S':
                self.key_linear = -self.linear_speed
                print("后退")
            elif key == 'a' or key == 'A':
                self.key_angular = self.angular_speed
                print("左转")
            elif key == 'd' or key == 'D':
                self.key_angular = -self.angular_speed
                print("右转")
            elif key == ' ':
                self.key_linear = 0.0
                self.key_angular = 0.0
                print("停止")
            elif key in ['1', '2', '3', '4', '5', '6', '7', '8', '9']:
                # 调整速度等级
                speed_level = int(key)
                self.linear_speed = 0.1 * speed_level
                self.angular_speed = 0.5 * speed_level
                print(f"速度等级设置为: {speed_level}")
            elif key == 'q' or key == 'Q':
                print("退出程序")
                self.keyboard_active = False
                rclpy.shutdown()
                break
            
    def publish_velocity(self):
        """发布速度指令"""
        twist_msg = Twist()
        twist_msg.linear.x = self.key_linear
        twist_msg.angular.z = self.key_angular
        self.vel_pub.publish(twist_msg)
        
    def cleanup(self):
        """清理资源"""
        self.keyboard_active = False
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    
    controller = RobotControllerSTM32()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.cleanup()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()