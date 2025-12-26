#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import serial
import numpy as np
import math
import threading
from collections import deque
import time
from pynput import keyboard

class FourWheelEKFNode(Node):
    def __init__(self):
        super().__init__('four_wheel_ekf_node')

        # ---------------- 1. 参数配置 ----------------
        self.declare_parameters(namespace='', parameters=[
            # 串口参数
            ('serial_port', '/dev/ttyUSB0'),
            ('baudrate', 115200),
            
            # 物理参数
            ('wheel_base', 0.18),  # 轴距 (m)
            ('wheel_track', 0.17), # 物理轮距 (m)
            ('wheel_radius', 0.0335), # 轮半径 (m)
            
            # 编码器脉冲数 (11线霍尔 * 30减速比 * 4倍频 = 1320)
            ('pulses_per_revolution', 1320.0), 
            
            # 滑移系数 (Skid Steer Slip Factor)
            ('skid_steer_slip_factor', 1.6),
            
            # EKF/ROS 参数
            ('ekf_frequency', 50.0),
            ('publish_tf', True),
            ('odom_frame_id', 'odom'),
            ('base_frame_id', 'base_link'),
            ('imu_frame_id', 'imu_link'),
            ('key_control_enabled', True)
        ])

        # 获取参数
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.wheel_track = float(self.get_parameter('wheel_track').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.pulses_per_rev = float(self.get_parameter('pulses_per_revolution').value)
        self.ekf_frequency = float(self.get_parameter('ekf_frequency').value)
        self.key_control_enabled = self.get_parameter('key_control_enabled').value
        self.skid_steer_slip_factor = float(self.get_parameter('skid_steer_slip_factor').value)

        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value

        # 计算有效轮距
        self.effective_track = self.wheel_track * self.skid_steer_slip_factor
        self.get_logger().info(f"物理轮距: {self.wheel_track}m, 滑移系数: {self.skid_steer_slip_factor}, 有效轮距: {self.effective_track:.3f}m")
# === 【修改点】添加线性修正系数 ===
        # 根据实测：Odom显示~1.27m，实际~2.6m -> 系数约 2.05
        self.linear_correction_factor = 1.236 
        
        # meters per encoder pulse
        self.distance_per_pulse = ((2.0 * math.pi * self.wheel_radius) / self.pulses_per_rev) * self.linear_correction_factor
        
        self.get_logger().info(f"线性修正系数: {self.linear_correction_factor}, 单脉冲距离: {self.distance_per_pulse:.6f}m")

        # ---------------- 状态定义: [x, y, theta, v, omega] ----------------
        self.state = np.zeros(5, dtype=float)
        
        # IMU 积分相关
        self.current_yaw = 0.0
        
        # ---------------- IMU 校准变量 ----------------
        self.imu_calibrated = False
        self.imu_calibration_samples = 0
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.calibration_target_samples = 100 
        
        # IMU 比例系数
        self.ACCEL_SCALE = 16384.0 / 9.8
        # 16.4 LSB/(deg/s) -> 转换为 rad/s
        self.GYRO_SCALE = 16.4 / (math.pi / 180.0) 

        # 数据缓冲区 (使用 deque)
        self.encoder_buffer = deque(maxlen=50)
        self.imu_buffer = deque(maxlen=100)

        # 串口与线程锁
        self.serial_conn = None
        self.serial_lock = threading.Lock()
        self.serial_buffer = ""
        self.lock = threading.Lock()

        # 键盘控制相关
        self.key_pressed = False
        self.current_key = None
        self.key_control_timer = None
        self.key_control_interval = 0.1

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 初始化串口
        self.init_serial()

        # 定时器
        self.ekf_timer = self.create_timer(1.0 / self.ekf_frequency, self.ekf_update)

        # 开启串口读取线程
        self.serial_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.serial_thread.start()

        # 初始化键盘控制
        if self.key_control_enabled:
            self.init_keyboard_control()

        self.get_logger().info('四轮差速融合节点已启动 (事件流排序版)')

    # ---------------- 串口处理核心逻辑 (修改部分) ----------------

    def parse_four_wheel_data(self, data):
        """
        解析函数：/four_wheel_encoder,<e1>,<e2>,<e3>,<e4>,<timestamp>
        """
        try:
            parts = data.split(',')
            if len(parts) >= 6:
                # 1. 解析四个轮子的脉冲增量
                delta_pulses = -np.array([int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4])], dtype=float)
                
                # 2. 获取 STM32 发送的毫秒级硬件时间戳
                current_timestamp_ms = int(parts[5])
                
                with self.lock:
                    # 初始化上一次时间戳
                    if not hasattr(self, 'last_encoder_timestamp_ms'):
                        self.last_encoder_timestamp_ms = current_timestamp_ms
                        return

                    # 计算硬件时间差 dt (毫秒)
                    dt_ms = current_timestamp_ms - self.last_encoder_timestamp_ms
                    self.last_encoder_timestamp_ms = current_timestamp_ms
                    
                    # 过滤无效时间差 (防止时间戳溢出或重复)
                    if dt_ms <= 0 or dt_ms > 200:
                        return

                    dt = dt_ms / 1000.0 # 转换为秒
                    
                    # 3. 计算物理速度
                    pulses_per_sec = delta_pulses / dt
                    wheel_linear_speed = pulses_per_sec * self.distance_per_pulse

                    # 4. 构建数据包 (包含 type 和 stm32_ts 用于混合排序)
                    encoder_data = {
                        'type': 'odom',                      # 数据类型标记
                        'timestamp': self.get_clock().now(), # ROS 接收时间 (仅用于 Header)
                        'stm32_ts': current_timestamp_ms,    # 【关键】硬件时间戳，用于排序
                        'wheel_speeds_m_s': wheel_linear_speed,
                        'dt': dt
                    }

                    self.encoder_buffer.append(encoder_data)
                    
        except Exception as e:
            self.get_logger().warning(f'编码器解析错误: {e}')

    def parse_imu_data(self, data):
        """
        解析函数：/imu_data,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<temp>,<timestamp>
        """
        try:
            parts = data.split(',')
            if len(parts) >= 9:
                # 1. 解析原始整数值
                raw_ax = int(parts[1]); raw_ay = int(parts[2]); raw_az = int(parts[3])
                raw_gx = int(parts[4]); raw_gy = int(parts[5]); raw_gz = int(parts[6])
                stm32_ts = int(parts[8]) # STM32发送的毫秒级时间戳
                
                current_time = self.get_clock().now()

                # ----------------- IMU 静止校准逻辑 -----------------
                if not self.imu_calibrated:
                    if self.imu_calibration_samples < self.calibration_target_samples:
                        self.accel_bias += np.array([raw_ax, raw_ay, raw_az])
                        self.gyro_bias += np.array([raw_gx, raw_gy, raw_gz])
                        self.imu_calibration_samples += 1
                        
                        if self.imu_calibration_samples == self.calibration_target_samples:
                            self.accel_bias /= self.calibration_target_samples
                            self.gyro_bias /= self.calibration_target_samples
                            self.accel_bias[2] = 0.0 # Z轴重力不校准
                            self.imu_calibrated = True
                            self.get_logger().info(f'IMU校准完成! Bias: GyroZ={self.gyro_bias[2]:.1f}')
                        return
                    return

                # ----------------- 物理量转换 -----------------
                # 原始数据减去零偏
                # 【注意】：Z轴取反以符合 ROS 右手坐标系 (逆时针为正)
                gyro_z_raw_corrected = raw_gz - self.gyro_bias[2]
                # 原系数 GYRO_SCALE 算出的角度偏大约 1.05 倍 (平均值)
                # 所以我们将分母扩大 1.05 倍，让结果变小
                calibration_factor = 1.05 
                corrected_scale = self.GYRO_SCALE * calibration_factor
                gyro_z_rad = - (gyro_z_raw_corrected / corrected_scale)

                # 加速度处理
                accel_x = (raw_ax - self.accel_bias[0]) / self.ACCEL_SCALE
                accel_y = (raw_ay - self.accel_bias[1]) / self.ACCEL_SCALE
                accel_z = (raw_az - self.accel_bias[2]) / self.ACCEL_SCALE

                # ----------------- 角度积分 -----------------
                # 初始化上一次的时间戳
                if not hasattr(self, 'last_imu_ts_ms'):
                    self.last_imu_ts_ms = stm32_ts
                    self.current_yaw = 0.0
                    return # 第一帧无法计算dt，跳过

                # 计算时间差
                dt_ms = stm32_ts - self.last_imu_ts_ms
                self.last_imu_ts_ms = stm32_ts

                # 过滤异常时间差
                if dt_ms <= 0 or dt_ms > 100: 
                    dt = 0.01 # 异常时回退到默认 10ms
                else:
                    dt = dt_ms / 1000.0

                # 角度积分: Yaw += 角速度 * 真实时间差
                self.current_yaw += gyro_z_rad * dt

                # 归一化到 -pi ~ pi
                self.current_yaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))

                # ----------------- 构建并存储消息 -----------------
                angular_velocity_msg = Vector3(x=0.0, y=0.0, z=gyro_z_rad)
                linear_acceleration_msg = Vector3(x=accel_x, y=accel_y, z=accel_z)
                orientation_quat = self.yaw_to_quaternion(self.current_yaw)

                with self.lock:
                    imu_entry = {
                        'type': 'imu',                              # 标记类型
                        'timestamp': current_time,
                        'stm32_ts': stm32_ts,                       # 【关键】硬件时间戳，用于排序
                        'orientation_yaw': self.current_yaw,        # 供 EKF 使用的浮点数角度
                        'orientation': orientation_quat,            # 供 发布 使用的四元数
                        'angular_velocity': angular_velocity_msg,
                        'linear_acceleration': linear_acceleration_msg
                    }
                    
                    self.imu_buffer.append(imu_entry)
                    
                    # 立即发布 IMU 话题
                    self.publish_imu_data(current_time, imu_entry)

        except Exception as e:
            self.get_logger().warning(f'IMU数据解析异常: {e}')

    # ---------------- 状态更新逻辑 (核心修改) ----------------

    def ekf_update(self):
        """
        融合更新：将一段时间内的 Odom 和 IMU 数据按时间顺序混合回放，解决时空错位问题。
        """
        current_time = self.get_clock().now()
        
        # 只要有数据就处理
        can_run = self.imu_calibrated and (len(self.encoder_buffer) > 0 or len(self.imu_buffer) > 0)

        if can_run:
            with self.lock:
                # 1. 取出两个缓冲区的所有数据
                events = []
                while self.encoder_buffer:
                    events.append(self.encoder_buffer.popleft())
                while self.imu_buffer:
                    events.append(self.imu_buffer.popleft())

            # 2. 【核心】按 STM32 硬件时间戳排序
            # 确保先发生的旋转先被处理，先发生的直行后被处理
            events.sort(key=lambda x: x['stm32_ts'])

            # 3. 顺序回放 (Event Replay)
            for event in events:
                if event['type'] == 'imu':
                    # 如果是 IMU 数据，更新当前已知的绝对角度
                    # 我们信任 IMU 的积分结果比编码器推算的旋转更准
                    self.state[2] = event['orientation_yaw']
                
                elif event['type'] == 'odom':
                    # 如果是里程计数据，使用 *当时* 的角度来计算位移
                    dt = event['dt']
                    ws = event['wheel_speeds_m_s']
                    
                    # 计算线速度
                    v_left = (ws[0] + ws[1]) / 2.0
                    v_right = (ws[2] + ws[3]) / 2.0
                    v = (v_left + v_right) / 2.0
                    
                    # 使用 state[2] (这个角度会被上面的 imu 事件不断更新)
                    # 这样就实现了：转弯时的位移沿切线方向，直行时的位移沿直线方向
                    self.state[0] += v * math.cos(self.state[2]) * dt
                    self.state[1] += v * math.sin(self.state[2]) * dt
                    self.state[3] = v # 更新速度状态

            self.last_ekf_time = current_time

        # 4. 发布最终计算出的里程计 TF
        self.publish_odometry(current_time)

    # ---------------- 辅助函数 ----------------

    def quaternion_to_yaw(self, quat):
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0; q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def publish_odometry(self, current_time):
        try:
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = self.odom_frame_id
            odom_msg.child_frame_id = self.base_frame_id

            # 位置
            odom_msg.pose.pose.position.x = float(self.state[0])
            odom_msg.pose.pose.position.y = float(self.state[1])
            odom_msg.pose.pose.position.z = 0.0
            # 姿态 (使用最新的 state[2])
            odom_msg.pose.pose.orientation = self.yaw_to_quaternion(float(self.state[2]))

            # 速度
            odom_msg.twist.twist.linear.x = float(self.state[3])
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.angular.z = float(self.state[4])

            # 协方差 (根据实际情况调整)
            pose_cov = [0.0] * 36
            pose_cov[0] = 0.05  # X
            pose_cov[7] = 0.05  # Y
            pose_cov[35] = 0.05 # Yaw
            
            twist_cov = [0.0] * 36
            twist_cov[0] = 0.02 # v
            twist_cov[35] = 0.05 # omega
            
            odom_msg.pose.covariance = pose_cov
            odom_msg.twist.covariance = twist_cov

            self.odom_pub.publish(odom_msg)

            if self.get_parameter('publish_tf').value:
                t = TransformStamped()
                t.header.stamp = odom_msg.header.stamp
                t.header.frame_id = self.odom_frame_id
                t.child_frame_id = self.base_frame_id
                t.transform.translation.x = float(self.state[0])
                t.transform.translation.y = float(self.state[1])
                t.transform.translation.z = 0.0
                t.transform.rotation = odom_msg.pose.pose.orientation
                self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f'发布里程计错误: {e}')

    def publish_imu_data(self, current_time, imu_data):
        try:
            imu_msg = Imu()
            imu_msg.header.stamp = current_time.to_msg()
            imu_msg.header.frame_id = self.imu_frame_id

            # 使用解析时计算好的四元数
            imu_msg.orientation = imu_data.get('orientation', self.yaw_to_quaternion(0.0))
            imu_msg.angular_velocity = imu_data['angular_velocity']
            imu_msg.linear_acceleration = imu_data['linear_acceleration']

            imu_msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            imu_msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
            imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

            self.imu_pub.publish(imu_msg)
        except Exception as e:
            self.get_logger().error(f'发布IMU数据错误: {e}')

    # ---------------- 串口通信部分 (保持不变) ----------------
    def init_serial(self):
        try:
            self.serial_conn = serial.Serial(port=self.serial_port,
                                             baudrate=self.baudrate,
                                             timeout=0.01,
                                             write_timeout=0.01,
                                             rtscts=False, dsrdtr=False)
            time.sleep(1.0)
            self.get_logger().info(f'成功连接到串口: {self.serial_port}, 波特率: {self.baudrate}')
            self.sync_stm32_time()
        except Exception as e:
            self.get_logger().error(f'无法打开串口: {e}')
            self.serial_conn = None
            self.get_logger().warning('串口未连接，将启用模拟数据')
            self.simulate_data()

    def sync_stm32_time(self):
        current_time_ms = int(time.time() * 1000)
        cmd = f"T{current_time_ms}\n"
        self.send_command_to_stm32(cmd, log_cmd=False)
        self.get_logger().info(f'发送时间同步命令: {cmd.strip()}')

    def serial_read_loop(self):
        if self.serial_conn is None:
            return

        while rclpy.ok():
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                    self.serial_buffer += data
                    while '\n' in self.serial_buffer:
                        line, self.serial_buffer = self.serial_buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self.parse_serial_data(line)
                time.sleep(0.001)
            except Exception as e:
                self.get_logger().error(f'串口读取错误: {e}')
                time.sleep(0.1)

    def parse_serial_data(self, data):
        try:
            if ']' in data:
                data = data.split(']')[-1].strip()

            if data.startswith('/four_wheel_encoder,'):
                self.parse_four_wheel_data(data)
            elif data.startswith('/imu_data,'):
                self.parse_imu_data(data)
            elif '系统状态' in data or '编码器' in data:
                self.get_logger().info(f'STM32状态: {data}')
        except Exception as e:
            self.get_logger().warning(f'数据解析错误: {data}, 错误: {e}')

    def send_command_to_stm32(self, command, log_cmd=True):
        if self.serial_conn is None or not self.serial_conn.is_open:
            if log_cmd:
                self.get_logger().warning('串口未连接，无法发送命令')
            return False
        try:
            with self.serial_lock:
                if command.startswith('T'):
                    full_command = command + '\n'
                elif len(command) == 1 and command in ['W', 'S', 'A', 'D', ' ']:
                    full_command = command
                else:
                    full_command = command + '\n'
                    
                self.serial_conn.write(full_command.encode('utf-8'))
                self.serial_conn.flush()
                if log_cmd:
                    self.get_logger().debug(f'发送命令到STM32: {full_command.strip()}', throttle_duration_sec=0.5)
                return True
        except Exception as e:
            self.get_logger().error(f'发送命令失败: {e}')
            return False

    # ---------------- 键盘控制 (保持不变) ----------------
    def init_keyboard_control(self):
        try:
            self.keyboard_listener = keyboard.Listener(
                on_press=self.on_key_press,
                on_release=self.on_key_release
            )
            self.keyboard_listener.daemon = True
            self.keyboard_listener.start()
            self.get_logger().info('键盘控制初始化成功，使用WASD控制移动，按ESC退出')
        except Exception as e:
            self.get_logger().error(f'键盘控制初始化失败: {e}')
            self.key_control_enabled = False

    def on_key_press(self, key):
        try:
            if key == keyboard.Key.esc:
                self.get_logger().info('收到ESC键，退出程序')
                try:
                    self.destroy_node()
                except:
                    pass
                rclpy.shutdown()
                return False

            if hasattr(key, 'char') and key.char is not None:
                kc = key.char.lower()
                if kc in ['w', 'a', 's', 'd']:
                    if not self.key_pressed or self.current_key != kc:
                        self.key_pressed = True
                        self.current_key = kc
                        self.send_key_command(kc)
                        self.start_key_control_timer()
                        self.get_logger().debug(f'按键按下: {kc}', throttle_duration_sec=1.0)
        except Exception as e:
            self.get_logger().warning(f'键盘按下事件处理错误: {e}')

    def on_key_release(self, key):
        try:
            if hasattr(key, 'char') and key.char is not None:
                kc = key.char.lower()
                if kc == self.current_key:
                    self.key_pressed = False
                    self.current_key = None
                    self.stop_movement()
                    self.stop_key_control_timer()
                    self.get_logger().debug('按键释放，机器人停止', throttle_duration_sec=1.0)
        except Exception as e:
            self.get_logger().warning(f'键盘释放事件处理错误: {e}')

    def send_key_command(self, key_char):
        command_map = {'w': 'W', 's': 'S', 'a': 'A', 'd': 'D'}
        cmd = command_map.get(key_char)
        if cmd:
            self.send_command_to_stm32(cmd)

    def start_key_control_timer(self):
        if self.key_control_timer is not None:
            self.key_control_timer.cancel()
        self.key_control_timer = threading.Timer(self.key_control_interval, self.key_control_timer_callback)
        self.key_control_timer.daemon = True
        self.key_control_timer.start()

    def key_control_timer_callback(self):
        if self.key_pressed and self.current_key is not None:
            self.send_key_command(self.current_key)
            self.start_key_control_timer()

    def stop_key_control_timer(self):
        if self.key_control_timer is not None:
            self.key_control_timer.cancel()
            self.key_control_timer = None

    def stop_movement(self):
        self.send_command_to_stm32(' ')

    def simulate_data(self):
        import threading
        def simulate_encoder():
            while rclpy.ok():
                delta_pulses = [np.random.randint(0, 5) for _ in range(4)]
                data = f"/four_wheel_encoder,{delta_pulses[0]},{delta_pulses[1]},{delta_pulses[2]},{delta_pulses[3]},{int(time.time()*1000)}\n"
                self.parse_serial_data(data.strip())
                time.sleep(0.1)

        def simulate_imu():
            while rclpy.ok():
                raw_ax = int(np.random.normal(0, 10))
                raw_ay = int(np.random.normal(0, 10))
                raw_az = int(np.random.normal(16384, 10))
                raw_gx = int(np.random.normal(0, 5))
                raw_gy = int(np.random.normal(0, 5))
                raw_gz = int(np.random.normal(0, 5))
                data = f"/imu_data,{raw_ax},{raw_ay},{raw_az},{raw_gx},{raw_gy},{raw_gz},25,{int(time.time()*1000)}\n"
                self.parse_serial_data(data.strip())
                time.sleep(0.02)

        encoder_thread = threading.Thread(target=simulate_encoder, daemon=True)
        encoder_thread.start()
        imu_thread = threading.Thread(target=simulate_imu, daemon=True)
        imu_thread.start()

    def destroy_node(self):
        self.get_logger().info('正在关闭四轮差速EKF节点...')
        self.stop_key_control_timer()
        if hasattr(self, 'keyboard_listener'):
            try:
                self.keyboard_listener.stop()
            except:
                pass
        self.stop_movement()
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FourWheelEKFNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n收到Ctrl+C，退出程序")
    except Exception as e:
        if node:
             node.get_logger().error(f"程序异常: {e}")
        else:
             print(f"程序异常: {e}")
    finally:
        if node:
            try:
                node.destroy_node()
            except:
                pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()