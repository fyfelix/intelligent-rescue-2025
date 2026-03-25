# config.py
"""
配置中心

本模块提供全局配置参数，同时保持与现有接口的兼容性。外部模块可通过以下属性访问配置：

属性：
    team (str): 当前选择的队伍，必须是 'blue' 或 'red'
    rescue_model (str): 模型权重文件路径
    label_balls (list): 根据当前队伍过滤后的球体标签列表
    label_area (list): 根据当前队伍过滤后的区域标签列表
"""

import os
from typing import Tuple

# ==================== 用户可配置参数 ====================
# 队伍配置
TEAM: str = ["blue", "red"][1]  # 当前队伍（默认选第一个）
RESCUE_MODEL = "./models/resecure2025.pt"             # 模型文件路径

# 视频输出配置
VIDEO_OUTPUT_DIR="./videos" # 视频保存目录
SAVE_OUTPUT: bool = True # 默认启用视频保存
VIDEO_CODEC: str = "mp4v"  # 视频编码格式
DEFAULT_FPS: int = 30  # 默认帧率
DEFAULT_RESOLUTION: Tuple[int, int] = (640, 480)  # 默认分辨率

# 区域坐标设置 -----------------------------> x1 y1 x2 y2
# 抓取区域 - 爪子能够成功抓取球体的区域
CATCH_AREA: Tuple[int, int, int, int] = (165, 430, 485, 460)

# 准备区域 - 开始执行靠近球体的区域
READY_AREA: Tuple[int, int, int, int] = (160, 178, 480, 388)

# 持球区域 - 成功抓取后球体应处的位置
HOLDING_AREA: Tuple[int, int, int, int] = (270, 380, 410, 460)

# 冲刺区域
CENTER_REGION: Tuple[int, int, int, int] = (250, 208, 390, 348)

CROSS_CENTER_REGION = (170, 430, 480, 460)

# 舵机角度设置 ----------------> 舵机1和2的角度
CATCH_ANGLE: Tuple[int, int] = (90,90)
RELEASE_ANGLE: Tuple[int, int] = (0,0)

# 串口设备口
UART_PORT: str = "/dev/ttyUSB0"
#UART_PORT: str = "COM9"

# ==================== 内部数据处理 ====================
_origin_label_balls = ["Red_Ball", "Blue_Ball", "Yellow_Ball", "Black_Ball"]
_origin_label_area = ["Blue_Placement_Zone", "Red_Placement_Zone"]
_ball_filter = {"blue": "Red_Ball", "red": "Blue_Ball"}
_area_filter = {"blue": "Red_Placement_Zone", "red": "Blue_Placement_Zone"}

label_balls = [x for x in _origin_label_balls if x != _ball_filter.get(TEAM, "")]
label_area = [x for x in _origin_label_area if x != _area_filter.get(TEAM, "")]

os.makedirs(VIDEO_OUTPUT_DIR, exist_ok=True)

def _validate():
    """参数安全检查"""
    assert TEAM in ("blue", "red"), "队伍必须是 'blue' 或 'red'"

# 初始化时自动校验
_validate()
