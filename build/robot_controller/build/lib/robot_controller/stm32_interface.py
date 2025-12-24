#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading
import time
import struct

class STM32Interface(Node):
    def __init__(self):
        super().__init__('stm32_interface')
        
        # 订阅者，接收控制指令
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # STM32通信参数
        self.serial_port = "/dev/ttyUSB0"  # 根据实际修改
        self.baudrate = 115200
        self.serial_timeout = 1.0
        
        # 控制参数
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # 串口对象
        self.ser = None
        
        # 初始化串口通信
        self.init_serial()
        
        # 创建定时器发送数据
        self.timer = self.create_timer(0.05, self.send_to_stm32)  # 20Hz
        
        self.get_logger().info("STM32接口已启动")
        
    def init_serial(self):
        """初始化串口通信"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=self.serial_timeout
            )
            self.get_logger().info(f"串口 {self.serial_port} 打开成功")
            
            # 发送初始化命令
            self.send_init_command()
            
        except Exception as e:
            self.get_logger().error(f"串口打开失败: {e}")
            self.ser = None
    
    def send_init_command(self):
        """发送初始化命令给STM32"""
        if self.ser and self.ser.is_open:
            try:
                # 发送初始化帧 (可以根据你的协议修改)
                init_frame = b'\xAA\x55\x00\x00\x00\x00\x00\x00\xFF'  # 示例帧
                self.ser.write(init_frame)
                self.get_logger().info("初始化命令已发送")
            except Exception as e:
                self.get_logger().error(f"发送初始化命令失败: {e}")
    
    def cmd_vel_callback(self, msg):
        """接收控制指令"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        
        # 限制速度范围
        self.linear_vel = max(min(self.linear_vel, 1.0), -1.0)
        self.angular_vel = max(min(self.angular_vel, 3.14), -3.14)
        
        self.get_logger().info(f"收到指令: 线速度={self.linear_vel:.2f}, 角速度={self.angular_vel:.2f}")
    
    def send_to_stm32(self):
        """发送数据给STM32"""
        if not self.ser or not self.ser.is_open:
            # 尝试重新连接
            self.init_serial()
            return
        
        try:
            # 方案1: 简单字符串协议
            # command = f"V{self.linear_vel:.2f},W{self.angular_vel:.2f}\n"
            # self.ser.write(command.encode())
            
            # 方案2: 二进制协议 (推荐)
            # 帧格式: 0xAA 0x55 linear_vel(H) linear_vel(L) angular_vel(H) angular_vel(L) checksum
            linear_int = int(self.linear_vel * 1000)  # 放大1000倍发送
            angular_int = int(self.angular_vel * 1000)
            
            # 限制范围
            linear_int = max(min(linear_int, 1000), -1000)
            angular_int = max(min(angular_int, 3140), -3140)
            
            # 构建数据帧
            frame = bytearray()
            frame.append(0xAA)  # 帧头
            frame.append(0x55)  # 帧头
            
            # 线速度 (2字节，有符号)
            frame.extend(linear_int.to_bytes(2, 'big', signed=True))
            
            # 角速度 (2字节，有符号)
            frame.extend(angular_int.to_bytes(2, 'big', signed=True))
            
            # 校验和
            checksum = sum(frame) & 0xFF
            frame.append(checksum)
            
            self.ser.write(frame)
            
            # 可选：读取STM32的响应
            if self.ser.in_waiting > 0:
                response = self.ser.read(self.ser.in_waiting)
                self.process_stm32_response(response)
                
        except Exception as e:
            self.get_logger().error(f"发送数据到STM32失败: {e}")
            self.ser = None
    
    def process_stm32_response(self, data):
        """处理STM32的响应数据"""
        # 这里可以添加处理STM32返回数据的逻辑
        # 例如：电池状态、编码器数据等
        if len(data) > 0:
            self.get_logger().info(f"收到STM32数据: {data.hex()}")
    
    def __del__(self):
        """析构函数，关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()

def main():
    rclpy.init()
    
    stm32_interface = STM32Interface()
    
    try:
        rclpy.spin(stm32_interface)
    except KeyboardInterrupt:
        pass
    finally:
        stm32_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()