#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 文件: uart.py
# 功能: 基于UART协议的电机与舵机控制核心模块
# 作者: 覃启轩，樊彧
# 团队: 北京建筑大学工程实践创新中心314工作室
# 创建日期: 2025-07-05
# -----------------------------------------------------------------------------
import serial
from serial.serialutil import SerialException
import threading
from typing import Optional, List
import time


class UARTController:
    def __init__(self):
        self._serial_port: Optional[serial.Serial] = None
        self._motor_speeds = [0, 0]  # 电机速度 [-100, 100]
        self._servo_angles = [0, 0]  # 舵机角度 [0, 360]
        self._latest_cmd = 0xAA  # 直接存储最新命令（覆盖旧数据）
        self._lock = threading.Lock()
        self._running = False  # 控制读取线程的运行状态
        self._read_thread = None  # 读取线程对象

    def _initialize_state(self):
        """初始化硬件状态"""
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)
        self.set_servo_angle(1, 0)
        self.set_servo_angle(2, 0)
        self.execute()
        time.sleep(0.5)  # 确保硬件响应

    def _init_uart(self, serial_port: str, baudrate: int) -> None:
        """内部串口初始化方法"""
        with self._lock:
            try:
                self._serial_port = serial.Serial(
                    port=serial_port,
                    baudrate=baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.5
                )
                print(f"UART 已连接: {serial_port}@{baudrate}bps")
                # 启动读取线程（直接覆盖模式）
                self._running = True
                self._read_thread = threading.Thread(
                    target=self._read_serial_data,
                    daemon=True
                )
                self._read_thread.start()
            except Exception as e:
                self._serial_port = None
                raise RuntimeError(f"串口初始化失败: {e}")

    def _read_serial_data(self):
        """持续读取串口数据（直接覆盖旧数据）"""
        while self._running and self._serial_port and self._serial_port.is_open:
            try:
                # 单字节读取（直接覆盖self._latest_cmd）
                data = self._serial_port.read(1)
                if data:
                    with self._lock:
                        self._latest_cmd = data[0]
                        # print(f"[RX] 收到命令: 0x{self._latest_cmd:02X}")  # 调试用
            except SerialException as e:
                print(f"[ERR] 串口读取错误: {e}")
                break
            except Exception as e:
                print(f"[ERR] 读取线程异常: {e}")
                break

    def get_latest_cmd(self) -> int:
        """获取最新接收到的命令（线程安全）"""
        with self._lock:
            return self._latest_cmd

    def init_uart(self, serial_port: str, baudrate: int = 115200) -> None:
        """初始化串口连接（失败时抛出异常）"""
        try:
            self._init_uart(serial_port, baudrate)
            self._initialize_state()
        except SerialException as e:
            raise RuntimeError(f"UART初始化失败: {e}") from e

    def close(self) -> None:
        """关闭串口连接"""
        with self._lock:
            self._running = False
            if self._read_thread and self._read_thread.is_alive():
                self._read_thread.join(timeout=1.0)

            if self._serial_port and self._serial_port.is_open:
                self._serial_port.close()
                print("UART 已关闭")

    @staticmethod
    def _build_pack(speed1: int, speed2: int, angle1: int, angle2: int) -> bytes:
        """
        构建完整数据包（带CRC校验）

        参数:
            speed1: 电机1速度 (-100~100)
            speed2: 电机2速度 (-100~100)
            angle1: 舵机1角度 (0~360)
            angle2: 舵机2角度 (0~360)

        返回:
            bytes: 完整数据包
        """
        pack = bytearray(13)  # 总长度变为13字节
        pack[0] = 0x55  # 包头
        pack[1] = 13  # 包长度更新为13

        # 处理速度值（加10000后拆分为2字节）
        speed1_val = speed1 + 10000  # 转换为正数范围
        pack[2] = (speed1_val >> 8) & 0xFF  # 速度1高字节
        pack[3] = speed1_val & 0xFF  # 速度1低字节

        speed2_val = speed2 + 10000
        pack[4] = (speed2_val >> 8) & 0xFF  # 速度2高字节
        pack[5] = speed2_val & 0xFF  # 速度2低字节

        # 处理舵机角度（保持2字节）
        pack[6] = (angle1 >> 8) & 0xFF  # 舵机1高字节
        pack[7] = angle1 & 0xFF  # 舵机1低字节
        pack[8] = (angle2 >> 8) & 0xFF  # 舵机2高字节
        pack[9] = angle2 & 0xFF  # 舵机2低字节

        # CRC计算范围扩展到[2,9]
        crc16 = UARTController._calculate_crc(pack, 2, 9)

        # CRC存放位置更新
        pack[10] = crc16 & 0xFF  # crc16低位
        pack[11] = (crc16 >> 8) & 0xFF  # crc16高位
        pack[12] = 0xAA  # 包尾

        return bytes(pack)

    def set_motor_speed(self, motor_id: int, speed: int):
        """设置电机速度"""
        self._motor_speeds[motor_id - 1] = speed

    def set_servo_angle(self, servo_id: int, angle: int):
        """设置舵机角度"""
        self._servo_angles[servo_id - 1] = angle

    def get_motor_speeds(self) -> List[int]:
        """获取当前电机速度"""
        return self._motor_speeds.copy()

    def get_servo_angles(self) -> List[int]:
        """获取当前舵机角度"""
        return self._servo_angles.copy()

    @staticmethod
    def _calculate_crc(data: bytes, start_byte: int, end_byte: int) -> int:
        """计算CRC校验值"""
        crc = 0xFFFF
        for i in range(start_byte, end_byte + 1):
            crc ^= data[i]
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def execute(self) -> bool:
        """发送当前状态的数据包"""
        with self._lock:
            if not self._serial_port or not self._serial_port.is_open:
                print("[ERR] UART 未就绪")
                return False

            try:
                packet = self._build_pack(
                    self._motor_speeds[0],
                    self._motor_speeds[1],
                    self._servo_angles[0],
                    self._servo_angles[1]
                )
                self._serial_port.write(packet)
                self._serial_port.flush()
                return True
            except SerialException as e:
                print(f"[ERR] 发送失败: {e}")

          

                return False


# ==============================测试程序==============================
def main1():
    packet = UARTController._build_pack(50,-50,100,100)
    print("数据包:\n", packet.hex(' ').upper())

def main2():
    import glob
    import sys

    def scan_serial_ports() -> List[str]:
        """扫描可用的串口设备"""
        ports = []
        # Windows
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        # Linux/Mac
        else:
            ports = glob.glob('/dev/tty[A-Za-z]*')

        available_ports = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                available_ports.append(port)
            except (OSError, serial.SerialException):
                pass
        return available_ports

    def main_menu() -> None:
        """主交互菜单（带串口扫描）"""
        while True:
            # 扫描可用串口
            ports = scan_serial_ports()
            if not ports:
                print("未检测到可用串口设备！")
                return

            print("\n===== 检测到以下串口 =====")
            print("0: 重新扫描串口")
            for i, port in enumerate(ports, 1):  # 从1开始编号
                print(f"{i}: {port}")

            # 手动选择串口
            selected_port=None
            while True:
                try:
                    choice = int(input("请选择要连接的串口编号: "))
                    if choice == 0:
                        break  # 跳出内层循环，重新扫描
                    selected_port = ports[choice - 1]  # 调整为0-based索引
                    break
                except (ValueError, IndexError):
                    print("无效选择，请重新输入")

            if choice == 0:
                continue  # 重新开始扫描

            # 初始化控制器
            controller = UARTController()
            try:
                print(f"\n正在连接 {selected_port}...")
                controller.init_uart(selected_port)
            except RuntimeError as e:
                print(f"连接失败: {e}")
                continue  # 返回菜单重新选择

            # 主控制循环
            while True:
                print("\n===== 控制菜单 =====")
                print("0. 退出程序并复位")
                print("1. 控制电机")
                print("2. 控制舵机")
                print("3. 复位所有电机")
                print("4. 复位所有舵机")
                print("5. 查看当前状态")

                try:
                    choice = input("请输入选项: ").strip()

                    if choice == "0":
                        # 退出前安全处理
                        print("\n正在复位所有设备...")
                        for mid in [1, 2]:
                            controller.set_motor_speed(mid, 0)
                        for sid in [1, 2]:
                            controller.set_servo_angle(sid, 0)
                        controller.execute()
                        controller.close()
                        print("程序已退出")
                        return  # 完全退出程序

                    elif choice == "1":
                        # 电机控制
                        try:
                            motor_id = int(input("电机ID (1-2): "))
                            if motor_id not in [1, 2]:
                                print("无效电机ID")
                                continue

                            speed = int(input("速度 (-100~100): "))
                            if not -100 <= speed <= 100:
                                print("速度超出范围")
                                continue

                            controller.set_motor_speed(motor_id, speed)
                            print(f"电机{motor_id} → 速度 {speed}%")

                        except ValueError:
                            print("请输入数字")

                    elif choice == "2":
                        # 舵机控制
                        try:
                            servo_id = int(input("舵机ID (1-2): "))
                            if servo_id not in [1, 2]:
                                print("无效舵机ID")
                                continue

                            angle = int(input("角度 (0~360): "))
                            if not 0 <= angle <= 360:
                                print("角度超出范围")
                                continue

                            controller.set_servo_angle(servo_id, angle)
                            print(f"舵机{servo_id} → 角度 {angle}°")

                        except ValueError:
                            print("请输入数字")

                    elif choice == "3":
                        # 复位电机
                        for mid in [1, 2]:
                            controller.set_motor_speed(mid, 0)
                            print(f"电机{mid} 已停止")

                    elif choice == "4":
                        # 复位舵机
                        for sid in [1, 2]:
                            controller.set_servo_angle(sid, 0)
                            print(f"舵机{sid} 已复位")

                    elif choice == "5":
                        # 状态查看
                        print("\n当前状态:")
                        for i, speed in enumerate(controller.get_motor_speeds(), 1):
                            print(f"电机{i}: {speed}%")
                        for i, angle in enumerate(controller.get_servo_angles(), 1):
                            print(f"舵机{i}: {angle}°")

                    else:
                        print("无效选项")

                    controller.execute()

                except KeyboardInterrupt:
                    print("\n正在中断操作...")
                    # 退出前安全处理
                    print("\n正在复位所有设备...")
                    for mid in [1, 2]:
                        controller.set_motor_speed(mid, 0)
                    for sid in [1, 2]:
                        controller.set_servo_angle(sid, 0)
                    controller.execute()
                    controller.close()
                    print("程序已退出")
                    return  # 完全退出程序

    main_menu()

def main3():
    import glob
    import sys

    def scan_serial_ports() -> List[str]:
        """扫描可用的串口设备"""
        ports = []
        # Windows
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        # Linux/Mac
        else:
            ports = glob.glob('/dev/tty[A-Za-z]*')

        available_ports = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                available_ports.append(port)
            except (OSError, serial.SerialException):
                pass
        return available_ports

    def main_menu() -> None:
        """主交互菜单（带串口扫描）"""
        while True:
            # 扫描可用串口
            ports = scan_serial_ports()
            if not ports:
                print("未检测到可用串口设备！")
                return

            print("\n===== 检测到以下串口 =====")
            print("0: 重新扫描串口")
            for i, port in enumerate(ports, 1):  # 从1开始编号
                print(f"{i}: {port}")

            # 手动选择串口
            selected_port = None
            while True:
                try:
                    choice = int(input("请选择要连接的串口编号: "))
                    if choice == 0:
                        break  # 跳出内层循环，重新扫描
                    selected_port = ports[choice - 1]  # 调整为0-based索引
                    break
                except (ValueError, IndexError):
                    print("无效选择，请重新输入")

            if choice == 0:
                continue  # 重新开始扫描

            # 初始化控制器
            controller = UARTController()
            try:
                print(f"\n正在连接 {selected_port}...")
                controller.init_uart(selected_port)
            except RuntimeError as e:
                print(f"连接失败: {e}")
                continue  # 返回菜单重新选择

            
            # 主控制循环
            while True:
                cmd = controller.get_latest_cmd()
                print("接收命令: 0x{:02X}".format(cmd))
            

    main_menu()

if __name__ == '__main__':
    #main1()
    main2()
    #main3() #只接收串口数据
