#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard
import threading
import time
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # ROS2订阅者，接收控制指令
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 发布者，用于发送实际控制指令
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_actual', 10)
        
        # 控制参数
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.control_mode = "keyboard"  # 或 "ros2"
        
        # 键盘控制参数
        self.key_linear = 0.0
        self.key_angular = 0.0
        self.linear_speed = 0.5  # 默认线速度
        self.angular_speed = 1.0  # 默认角速度
        
        # 键盘监听
        self.listener = None
        self.keyboard_active = False
        
        # 创建定时器发布控制指令
        self.timer = self.create_timer(0.1, self.publish_velocity)  # 10Hz
        
        self.get_logger().info("机器人控制器已启动 - 模式: 键盘控制")
        
    def cmd_vel_callback(self, msg):
        """接收ROS2控制指令"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        self.control_mode = "ros2"
        
    def start_keyboard_control(self):
        """启动键盘控制"""
        self.control_mode = "keyboard"
        self.keyboard_active = True
        self.listener = keyboard.Listener(
            on_press=self.on_key_press,
            on_release=self.on_key_release
        )
        self.listener.start()
        self.get_logger().info("键盘控制已启动")
        
    def stop_keyboard_control(self):
        """停止键盘控制"""
        self.keyboard_active = False
        if self.listener:
            self.listener.stop()
        self.key_linear = 0.0
        self.key_angular = 0.0
        self.get_logger().info("键盘控制已停止")
        
    def on_key_press(self, key):
        """键盘按下事件"""
        if not self.keyboard_active:
            return
            
        try:
            if key == keyboard.Key.up:
                self.key_linear = self.linear_speed
            elif key == keyboard.Key.down:
                self.key_linear = -self.linear_speed
            elif key == keyboard.Key.left:
                self.key_angular = self.angular_speed
            elif key == keyboard.Key.right:
                self.key_angular = -self.angular_speed
            elif key == keyboard.Key.space:
                # 急停
                self.key_linear = 0.0
                self.key_angular = 0.0
            elif hasattr(key, 'char'):
                if key.char == 'w' or key.char == 'W':
                    self.key_linear = self.linear_speed
                elif key.char == 's' or key.char == 'S':
                    self.key_linear = -self.linear_speed
                elif key.char == 'a' or key.char == 'A':
                    self.key_angular = self.angular_speed
                elif key.char == 'd' or key.char == 'D':
                    self.key_angular = -self.angular_speed
                elif key.char == 'q' or key.char == 'Q':
                    # 退出
                    self.stop_keyboard_control()
                    rclpy.shutdown()
                    return
        except AttributeError:
            pass
            
    def on_key_release(self, key):
        """键盘释放事件"""
        if not self.keyboard_active:
            return
            
        try:
            if key in [keyboard.Key.up, keyboard.Key.down, 
                      keyboard.Key.left, keyboard.Key.right]:
                if key in [keyboard.Key.up, keyboard.Key.down]:
                    self.key_linear = 0.0
                if key in [keyboard.Key.left, keyboard.Key.right]:
                    self.key_angular = 0.0
            elif hasattr(key, 'char'):
                if key.char in ['w', 'W', 's', 'S']:
                    self.key_linear = 0.0
                elif key.char in ['a', 'A', 'd', 'D']:
                    self.key_angular = 0.0
        except AttributeError:
            pass
            
    def publish_velocity(self):
        """发布速度指令"""
        twist_msg = Twist()
        
        if self.control_mode == "keyboard":
            # 键盘控制模式
            twist_msg.linear.x = self.key_linear
            twist_msg.angular.z = self.key_angular
        else:
            # ROS2控制模式
            twist_msg.linear.x = self.linear_vel
            twist_msg.angular.z = self.angular_vel
            
        self.vel_pub.publish(twist_msg)
        
    def set_speed(self, linear_speed, angular_speed):
        """设置速度参数"""
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.get_logger().info(f"速度设置: 线速度={linear_speed}, 角速度={angular_speed}")
        
    def switch_to_ros2_control(self):
        """切换到ROS2控制模式"""
        self.control_mode = "ros2"
        self.get_logger().info("已切换到ROS2控制模式")
        
    def switch_to_keyboard_control(self):
        """切换到键盘控制模式"""
        self.control_mode = "keyboard"
        self.get_logger().info("已切换到键盘控制模式")

def main():
    rclpy.init()
    
    # 创建控制器节点
    controller = RobotController()
    
    try:
        # 启动键盘控制
        controller.start_keyboard_control()
        
        # 打印控制说明
        print("\n=== 键盘控制说明 ===")
        print("方向键/WASD: 控制移动")
        print("空格键: 急停")
        print("Q: 退出程序")
        print("===================\n")
        
        # 保持节点运行
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        controller.stop_keyboard_control()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()