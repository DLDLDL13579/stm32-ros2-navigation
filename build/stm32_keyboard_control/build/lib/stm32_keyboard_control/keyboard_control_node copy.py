#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
import sys
import select
import tty
import termios
import time
import threading

class TurtleStyleKeyboardControl(Node):
    """
    基于小海龟方案的键盘控制节点
    使用termios和select实现可靠的键盘输入
    """

    def __init__(self):
        super().__init__('turtle_style_keyboard_control')
        
        # 声明节点参数
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)

        # 获取参数值
        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value

        # 初始化串口连接
        try:
            self.serial_conn = serial.Serial(
                port=serial_port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout
            )
            time.sleep(2)
            self.get_logger().info(f'成功连接到STM32，串口: {serial_port}')
        except Exception as e:
            self.get_logger().error(f'无法打开串口 {serial_port}: {e}')
            raise e

        # 初始化状态变量
        self.led_state = False
        self.is_running = True
        
        # 保存终端原始设置
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self._keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.get_logger().info('小海龟风格键盘监听器已启动')
        self._print_instructions()

    def _print_instructions(self):
        """打印操作说明"""
        instructions = """
=== LED键盘控制说明 ===
按键功能:
  '1'     - 切换LED开关状态
  '↑'     - 切换LED开关状态 (上箭头键)
  'ESC'   - 退出程序
===================
        """
        print(instructions)

    def _keyboard_listener(self):
        """
        核心键盘监听函数 - 基于小海龟方案
        使用termios原始模式和select非阻塞读取
        """
        try:
            # 设置终端为原始模式，立即读取字符而不等待回车
            tty.setraw(sys.stdin.fileno())
            
            self.get_logger().info("键盘监听线程已启动")
            
            while self.is_running and rclpy.ok():
                # 使用select实现非阻塞读取，超时时间100ms
                read_ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                
                if read_ready:
                    # 读取按键
                    key = sys.stdin.read(1)
                    
                    # 处理ESC键（ASCII 27）
                    if key == '\x1b':
                        # 检查是否是箭头键（以ESC [开头）
                        read_ready, _, _ = select.select([sys.stdin], [], [], 0.01)
                        if read_ready:
                            next_char = sys.stdin.read(1)
                            if next_char == '[':
                                third_char = sys.stdin.read(1)
                                if third_char == 'A':  # 上箭头
                                    self._handle_key_press('UP_ARROW')
                            else:
                                # 单独的ESC键 - 退出程序
                                self.get_logger().info("检测到ESC键，退出程序")
                                self.is_running = False
                                break
                    else:
                        # 处理普通按键
                        self._handle_key_press(key)
                
        except Exception as e:
            self.get_logger().error(f'键盘监听错误: {e}')
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            self.get_logger().info("键盘监听线程已退出")

    def _handle_key_press(self, key):
        """处理按键事件"""
        if key == '1':
            self.get_logger().info("检测到按键 '1'")
            self.toggle_led()
        elif key == 'UP_ARROW':
            self.get_logger().info("检测到上箭头键")
            self.toggle_led()
        else:
            # 忽略其他按键，但可以记录调试信息
            self.get_logger().debug(f'忽略按键: {repr(key)}')

    def toggle_led(self):
        """切换LED状态"""
        if self.led_state:
            command = "LED_OFF"
            self.led_state = False
            state_text = "关闭LED"
        else:
            command = "LED_ON"
            self.led_state = True
            state_text = "打开LED"

        self.get_logger().info(f"{state_text}")
        self.send_led_command(command)

    def send_led_command(self, command):
        """发送LED控制命令"""
        try:
            full_command = command + '\n'
            self.get_logger().info(f'发送指令: "{command}"')
            
            bytes_written = self.serial_conn.write(full_command.encode('utf-8'))
            self.serial_conn.flush()
            
            self.get_logger().debug(f'成功发送 {bytes_written} 字节到串口')
            
        except Exception as e:
            self.get_logger().error(f'发送指令 "{command}" 失败: {e}')

    def destroy_node(self):
        """节点清理"""
        self.get_logger().info("正在进行清理工作...")
        
        # 设置停止标志
        self.is_running = False
        
        # 等待键盘线程退出
        if hasattr(self, 'keyboard_thread') and self.keyboard_thread.is_alive():
            self.keyboard_thread.join(timeout=1.0)
        
        # 发送最终关闭指令
        if hasattr(self, 'serial_conn') and self.serial_conn.is_open:
            try:
                self.send_led_command("LED_OFF")
                time.sleep(0.1)
            except:
                pass
            finally:
                self.serial_conn.close()
        
        # 确保终端设置被恢复
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        except:
            pass
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TurtleStyleKeyboardControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到Ctrl+C信号，节点即将关闭...")
    except Exception as e:
        if 'node' in locals():
            node.get_logger().error(f'节点运行出错: {e}')
        else:
            print(f'节点初始化失败: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()