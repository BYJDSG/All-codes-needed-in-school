# 功能：
# 分为两种模式：
# 1. 张开手，上抬飞机起飞（上升），下放下降，前后移动代表飞机前后移动，左右摆动手套代表航向角的变化，左右移动手套代表飞机左右移动
# 2. 握拳后，伸出食指代表飞机在空中画圆，伸出中指代表飞机降落，伸出拇指代表飞机向前冲刺10米。

# 分工：
# yyk：手套imu部分代码
# fhr：手套电位计部分代码
# byj：PC端
# gcx：串口助手

# 假设信息顺序:MODE, X_acc(右正), Y_acc(前正), Z_acc(上正), gloveRoll(逆正), glovePitch(逆正), gloveYaw(逆正)

# Codes:

# command_counter = 0
# times_need_to_send = 50
# command_sequence = []  #填充各种对应操作的sequence
import math
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander
import serial
import matplotlib.pyplot as plt
import cv2

signal_history = []
max_history_length = 10
required_stable_count = 6
sequence1 = []
sequence2 = []

global position_estimate
global euler_estimate
position_estimate = [0, 0, 0]
euler_estimate = [0, 0, 0]
global pos_x,pos_y,pos_z
pos_x = []
pos_y = []
pos_z = []
global roll,pitch,yaw
roll = []
pitch =[]
yaw = []
global cf_timestamp
cf_timestamp = []

actions = {
    (1, True, True, True, True, False): sequence1,
    (2, False, True, False, True, True): sequence2,
    # 更多的信号模式和对应的行动
}

def find_stable_signal(signal_history, required_stable_count):
    if len(signal_history) < required_stable_count:
        return None

    # 检查每个可能的信号模式是否在历史中稳定
    for signal_pattern in set(signal_history):
        if signal_history.count(signal_pattern) >= required_stable_count:
            return signal_pattern
    return None

def update_signal_history(signal_history, new_signal, max_history_length):
    # 在列表的开始位置插入新的信号
    signal_history.insert(0, tuple(new_signal))
    
    # 如果历史记录的长度超过了设定的最大长度，则移除最旧的信号（列表末尾的元素）
    if len(signal_history) > max_history_length:
        signal_history.pop()

def read_signal():
    data_from_com = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=1)
    mode = data_from_com[0]
    X_acc = True if data_from_com[1] > 0.05  else False
    Y_acc = True if data_from_com[2] > 0.05  else False
    Z_acc = True if data_from_com[3] > 0.05  else False
    gloveRoll = True if data_from_com[4] > 100 else False
    glovePitch = True if data_from_com[5] > 100 else False
    gloveYaw = True if data_from_com[6] > 100 else False
    return (mode, X_acc, Y_acc, Z_acc, gloveRoll, glovePitch, gloveYaw)

def run_sequence(scf, sequence, base_x, base_y, base_z, yaw):
    cf = scf.cf

    for position in sequence:
        print('Setting position {}'.format(position))

        x = position[0] + base_x
        y = position[1] + base_y
        z = position[2] #+ base_z * 0.0

        #important!! needs init
        # cf.commander.send_setpoint(roll = 0, pitch = 0, yawrate = 0, thrust = 0)
        for i in range(500):
            # change the following line#
            cf.commander.send_hover_setpoint(vx = 0.2 * (x - position_estimate[0]), vy =  0.2 * (y - position_estimate[1]), yawrate = 0, zdistance = 0.5)
            
            
            time.sleep(0.05)
            cf.commander.send_zdistance_setpoint(roll = -0.06*euler_estimate[0], pitch = -0.06*euler_estimate[1], yawrate = - euler_estimate[2], zdistance = 0.5)
            time.sleep(0.1)
            x_for_first_command = position_estimate[0]
            y_for_first_command = position_estimate[1]
            print(i)
            
    cf.commander.send_setpoint(roll = 0, pitch = 0, yawrate = 0, thrust = 0) #compensate for error in rp
    time.sleep(1)
    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(1)


while True:
    current_signal = read_signal()  # 模拟读取当前信号
    update_signal_history(signal_history, current_signal, max_history_length)

    stable_signal = find_stable_signal(signal_history, required_stable_count)
    if stable_signal and stable_signal in actions:
        run_sequence(actions[stable_signal]) # 执行与稳定信号对应的行动
    else:
        run_sequence(sequence1)



