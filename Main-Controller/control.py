#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 文件: control.py
# 功能: 运动控制核心模块
# 团队: 北京建筑大学工程实践创新中心314工作室
# 作者: 樊彧，覃启轩
# 创建日期: 2025-07-05
# -----------------------------------------------------------------------------
from uart import UARTController
import signal
import time
import sys
import atexit
from config import CATCH_ANGLE, RELEASE_ANGLE, UART_PORT, DEFAULT_RESOLUTION

frame_width, frame_height = DEFAULT_RESOLUTION


class Controller(UARTController):
    def __init__(self, port: str = UART_PORT, baudrate: int = 115200):
        """
        运动控制器 (依赖底层UARTController的范围检查)

        参数:
            port: 串口设备路径 (默认从config导入)
            baudrate: 波特率 (默认115200)
        """
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self._serial_port = None  # 显式初始化

        # 注册安全钩子
        self.flag_cleanup = False
        signal.signal(signal.SIGINT, self._emergency_stop)
        signal.signal(signal.SIGTERM, self._emergency_stop)
        atexit.register(self._safe_shutdown)

        try:
            self._init_uart(self.port, self.baudrate)
            self._initialize_state()
        except Exception as e:
            self._safe_shutdown()  # 清理可能残留的状态
            raise RuntimeError(f"初始化失败: {e}") from e

    def _initialize_state(self) -> None:
        """初始化硬件状态"""
        self.stop()  # 停止所有电机
        self.release()  # 复位舵机
        time.sleep(0.3)  # 确保硬件响应

    def _safe_shutdown(self) -> None:
        """线程安全的关闭方法"""
        if self.flag_cleanup:
            return
        self.flag_cleanup = True
        print("\n执行安全复位协议...")
        try:
            # 1. 停止电机（不依赖串口状态）
            if hasattr(self, '_motor_speeds'):
                self.stop()
                time.sleep(0.1)

            # 2. 复位舵机（不依赖串口状态）
            if hasattr(self, '_servo_angles'):
                self.release()
                time.sleep(0.1)

            # 3. 关闭串口（严格检查）
            if self._serial_port is not None:
                if hasattr(self._serial_port, 'is_open'):
                    self._serial_port.close()
                self._serial_port = None
        except Exception as e:
            print(f"[WARN] 安全关闭时出错: {str(e)}")
        finally:
            print("硬件复位完成")

    def _emergency_stop(self, signum=None, frame=None) -> None:
        """信号中断处理"""
        print(f"\n捕获到中断信号 {signum}，紧急停止...")
        self._safe_shutdown()

        sys.exit(1)

    def execute(self):
        """执行操作并自动处理连接异常"""
        MAX_RETRIES = 3  # 最大重试次数
        RETRY_DELAY = 0.5  # 重试延迟(秒)

        try:
            super().execute()
        except Exception as e:
            print(f"[WARN] 执行失败: {str(e)}")
            for attempt in range(MAX_RETRIES):
                try:
                    time.sleep(RETRY_DELAY)
                    self.__init__(self.port, self.baudrate)
                    print(f"[INFO] 第{attempt + 1}次重连成功")
                    return True
                except Exception as e:
                    print(f"[WARN] 第{attempt + 1}次重连失败: {str(e)}")
                    if attempt == MAX_RETRIES - 1:
                        print("[ERR] 达到最大重试次数")
                        return False
            return False

    def __del__(self):
        """对象销毁时自动复位"""
        self._safe_shutdown()

    def close(self) -> None:
        super().close()
        self._safe_shutdown()

    def forward(self, speed) -> None:
        """前进"""
        self.set_motor_speed(1, -speed)
        self.set_motor_speed(2, -speed)
        self.execute()

    def backward(self, speed) -> None:
        """后退"""
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)
        self.execute()

    def left(self, speed) -> None:
        """左转"""
        self.set_motor_speed(1, -speed)
        self.set_motor_speed(2, speed)
        self.execute()

    def right(self, speed) -> None:
        """右转"""
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, -speed)
        self.execute()

    def stop(self) -> None:
        """停止所有电机"""
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)
        self.execute()

    # 舵机爪子张开和闭合的角度去config.py里调整，不要在这里调整
    def catch(self, angle1=CATCH_ANGLE[0], angle2=CATCH_ANGLE[1]) -> None:
        """执行抓取动作"""
        self.set_servo_angle(1, angle1)
        self.set_servo_angle(2, angle2)
        self.execute()
        time.sleep(0.3)  # 等待动作完成

    # 舵机爪子张开和闭合的角度去config.py里调整，不要在这里调整
    def release(self, angle1=RELEASE_ANGLE[0], angle2=RELEASE_ANGLE[1]) -> None:
        """执行释放动作"""
        self.set_servo_angle(1, angle1)
        self.set_servo_angle(2, angle2)
        self.execute()
        time.sleep(0.3)  # 等待动作完成

    def start_catch(self, angle1=CATCH_ANGLE[0], angle2=CATCH_ANGLE[1]) -> None:
        """执行抓取动作"""
        self.set_servo_angle(1, 90)
        self.execute()
        time.sleep(0.3)  # 等待动作完成
        self.set_servo_angle(1, 0)
        self.execute()
        time.sleep(0.3)  # 等待动作完成

    def approach_ball(self, x, y) -> None:
        """根据球体位置调整运动方向，使用PID控制实现精确转向"""
        Kp = 0.15  # 比例系数,与响应速度成正比
        Ki = 0.001  # 积分系数，与消除误差速度成正比
        Kd = 0.03  # 微分系数，抑制震荡效果

        # 图像中心坐标
        center_x = 320
        center_y = frame_height / 2

        # 计算球在水平方向上的偏移量（误差）
        error_x = x - center_x

        # 计算速度比例因子（0-1之间），基于垂直位置
        if y > (frame_height * 0.75):  # 底部区域（靠近）
            base_speed = 4.0
            speed_factor = 0.5
            turn_factor = 1.0
        elif y > (frame_height * 0.5):  # 中部区域
            base_speed = 4.0
            speed_factor = 0.7
            turn_factor = 1.5
        else:  # 顶部区域（远离）
            base_speed = 5.5
            speed_factor = 1.0
            turn_factor = 2.0

        # 初始化或更新 PID 状态
        if not hasattr(self, '_pid_integral'):
            self._pid_integral = 0.0
            self._pid_last_error = 0.0
            self._pid_last_time = time.time()

        # 计算时间差
        current_time = time.time()
        dt = current_time - self._pid_last_time
        self._pid_last_time = current_time

        # 比例项
        P = Kp * error_x

        # 积分项（防止积分饱和）
        self._pid_integral += error_x * dt
        # 积分限幅
        I = Ki * min(max(self._pid_integral, -100), 100)

        # 微分项
        derivative = (error_x - self._pid_last_error) / dt if dt > 0 else 0
        D = Kd * derivative
        self._pid_last_error = error_x

        # PID 总输出
        pid_output = P + I + D

        # 限幅处理，确保输出在合理范围内
        max_correction = base_speed * 0.8  # 最大修正量为基速的80%
        correction = max(min(pid_output, max_correction), -max_correction)

        # 调整两侧电机速度实现精确转向
        left_speed = base_speed + correction
        right_speed = base_speed - correction

        # 应用速度因子
        left_speed *= speed_factor
        right_speed *= speed_factor

        # 将速度转换为两位整数+两位小数的形式
        left_speed_formatted = int(left_speed * 100)
        right_speed_formatted = int(right_speed * 100)

        # 确保最低速度不低于400（绝对值）
        min_speed = 400  # 4.00

        # 处理左轮速度
        if abs(left_speed_formatted) < min_speed:
            # 保持方向但确保最小速度
            if left_speed_formatted >= 0:
                left_speed_formatted = min_speed
            else:
                left_speed_formatted = -min_speed

        # 处理右轮速度
        if abs(right_speed_formatted) < min_speed:
            # 保持方向但确保最小速度
            if right_speed_formatted >= 0:
                right_speed_formatted = min_speed
            else:
                right_speed_formatted = -min_speed

        # 设置电机速度
        self.set_motor_speed(1, -left_speed_formatted)
        self.set_motor_speed(2, -right_speed_formatted)

        # 记录调试信息
        if pid_output != 0:
            print(f"PID校正: {correction:.2f} (P:{P:.2f} I:{I:.2f} D:{D:.2f})")
        print(f"球位置: ({x:.1f}, {y:.1f}) | 左轮: {-left_speed_formatted} | 右轮: {-right_speed_formatted}")

        self.execute()

    def approach_area(self, x, y) -> None:
        Kp = 0.12  # 比例系数,与响应速度成正比
        Ki = 0.001  # 积分系数，与消除误差速度成正比
        Kd = 0.03  # 微分系数，抑制震荡效果

        # 图像中心坐标
        center_x = 320
        center_y = frame_height / 2

        # 计算球在水平方向上的偏移量（误差）
        error_x = x - center_x

        # 计算速度比例因子（0-1之间），基于垂直位置
        if y > (frame_height * 0.75):  # 底部区域（靠近）
            base_speed = 4.0
            speed_factor = 0.5
            turn_factor = 1.0
        elif y > (frame_height * 0.5):  # 中部区域
            base_speed = 4.0
            speed_factor = 0.7
            turn_factor = 1.5
        else:  # 顶部区域（远离）
            base_speed = 5.5
            speed_factor = 1.0
            turn_factor = 2.0

        # 初始化或更新 PID 状态
        if not hasattr(self, '_pid_integral'):
            self._pid_integral = 0.0
            self._pid_last_error = 0.0
            self._pid_last_time = time.time()

        # 计算时间差
        current_time = time.time()
        dt = current_time - self._pid_last_time
        self._pid_last_time = current_time

        # 比例项
        P = Kp * error_x

        # 积分项（防止积分饱和）
        self._pid_integral += error_x * dt
        # 积分限幅
        I = Ki * min(max(self._pid_integral, -100), 100)

        # 微分项
        derivative = (error_x - self._pid_last_error) / dt if dt > 0 else 0
        D = Kd * derivative
        self._pid_last_error = error_x

        # PID 总输出
        pid_output = P + I + D

        # 限幅处理，确保输出在合理范围内
        max_correction = base_speed * 0.8  # 最大修正量为基速的80%
        correction = max(min(pid_output, max_correction), -max_correction)

        # 调整两侧电机速度实现精确转向
        left_speed = base_speed + correction
        right_speed = base_speed - correction

        # 应用速度因子
        left_speed *= speed_factor
        right_speed *= speed_factor

        # 将速度转换为两位整数+两位小数的形式
        left_speed_formatted = int(left_speed * 100)
        right_speed_formatted = int(right_speed * 100)

        # 确保最低速度不低于400（绝对值）
        min_speed = 400  # 4.00

        # 处理左轮速度
        if abs(left_speed_formatted) < min_speed:
            # 保持方向但确保最小速度
            if left_speed_formatted >= 0:
                left_speed_formatted = min_speed
            else:
                left_speed_formatted = -min_speed

        # 处理右轮速度
        if abs(right_speed_formatted) < min_speed:
            # 保持方向但确保最小速度
            if right_speed_formatted >= 0:
                right_speed_formatted = min_speed
            else:
                right_speed_formatted = -min_speed

        # 设置电机速度
        self.set_motor_speed(1, -left_speed_formatted)
        self.set_motor_speed(2, -right_speed_formatted)

        self.execute()

    def search_cross(self, speed) -> None:
        "搜索十字"
        self.left(speed)

    def search_ball(self, speed) -> None:
        """搜索球体"""
        self.left(speed)

    def search_area(self, speed) -> None:
        """搜索区域"""
        self.right(speed)


# ===================== 测试程序 =====================

# 电机测试
def main1():
    print("开始测试电机")
    controller = Controller()
    time.sleep(1)

    print("前进")
    controller.forward(1200)
    time.sleep(3)
    controller.stop()
    time.sleep(1)

    print("后退")
    controller.backward(1200)
    time.sleep(3)
    controller.stop()
    time.sleep(1)

    print("左转")
    controller.left(1200)
    time.sleep(3)
    controller.stop()
    time.sleep(1)

    print("右转")
    controller.right(1200)
    time.sleep(3)
    controller.stop()
    time.sleep(1)

    controller.close()
    print("电机测试完毕\n")


# 舵机测试
def main2():
    print("开始测试舵机")
    controller = Controller()
    time.sleep(1)

    print("爪子闭合")
    controller.catch()
    time.sleep(1)
    print("爪子张开")
    controller.release()
    time.sleep(1)
    print("爪子闭合")
    controller.catch()
    time.sleep(1)
    print("爪子张开")
    controller.release()
    time.sleep(1)

    controller.close()
    print("舵机测试完毕\n")


if __name__ == '__main__':
    main1()
    main2()
