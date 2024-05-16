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
import numpy as np
# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
signal_history = []
max_history_length = 10
required_stable_count = 6
sequence1 = [0,0,0.5]
sequence2 = [0,0,0.6]
sequence3 = [0,0,0.4]
sequence0 = [0,0,0.1]
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
global loop_run
loop_run = False

actions = {
    (0, 0, 0, 0, 0, 0, 0): sequence1,
    (0, 0, 0, 1, 1, 0, 0): sequence2,
    (0, 0, 0, 1, 0, 0, 0): sequence3,
    (1, 0, 0, 0, 0, 0, 0): sequence0,
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

def read_signal(bluetoothSerial):
    # 忽略第一个数据包
    initial_line = bluetoothSerial.readline().decode('utf-8').rstrip()
    # print("忽略第一个数据包:", initial_line)
    
    while True:
        if bluetoothSerial.inWaiting() > 0:
            # 读取数据包
            data_line = bluetoothSerial.readline().decode('utf-8').rstrip()
            data_from_com = data_line.split(',')
            # 确保接收到的数据包长度是7（完整的数据包）
            if len(data_from_com) == 7:
                try:
                    mode = int(float(data_from_com[0]))
                    X_acc = 1 if float(data_from_com[1]) > 0.05 else -1 if float(data_from_com[1]) < -0.05 else 0
                    Y_acc = 1 if float(data_from_com[2]) > 0.05 else -1 if float(data_from_com[2]) < -0.05 else 0
                    Z_acc = 1 if float(data_from_com[3]) > 0.05 else -1 if float(data_from_com[3]) < -0.05 else 0
                    gloveRoll = 1 if float(data_from_com[4]) > 100 else 0
                    glovePitch = 1 if float(data_from_com[5]) > 100 else 0
                    gloveYaw = 1 if float(data_from_com[6]) > 100 else 0
                    return (mode, X_acc, Y_acc, Z_acc, gloveRoll, glovePitch, gloveYaw)
                except ValueError:
                    print("数据格式错误:", data_from_com)
            else:
                print("收到不完整的数据包:", data_from_com)
        time.sleep(0.1)

def run_sequence(scf, sequence, base_x, base_y, base_z, yaw):
    cf = scf.cf

    for position in sequence:
        print('Setting position {}'.format(position))

        x = position[0] #+ base_x
        y = position[1] #+ base_y
        z = position[2] 

        #important!! needs init
        # cf.commander.send_setpoint(roll = 0, pitch = 0, yawrate = 0, thrust = 0)
        for i in range(10): #fly
            x_for_first_command = position_estimate[0]
            y_for_first_command = position_estimate[1]
            # change the following line#
            # cf.commander.send_hover_setpoint(vx = 0.5 * (x - x_for_first_command), vy =  0.5 * (y - y_for_first_command), yawrate = 0, zdistance = 0.5)

            roll_cmd = 35.5 * (y - y_for_first_command)
            pitch_cmd = 80.5 * (x - x_for_first_command)

            roll_cmd = np.clip(roll_cmd, -3.0,3.0)
            pitch_cmd = np.clip(pitch_cmd, -3.0,3.0)
            #x back = positive
            #y right positive
            #pitch head up positive
            #roll right down positve
            cf.commander.send_zdistance_setpoint(roll = roll_cmd, pitch = pitch_cmd, yawrate = 0, zdistance = z)
            time.sleep(0.15)
            # cf.commander.send_zdistance_setpoint(roll = -0.75*euler_estimate[0], pitch = -0.75*euler_estimate[1], yawrate = -0.3 * euler_estimate[2], zdistance = z)
            # time.sleep(0.05)
            cf.commander.send_zdistance_setpoint(roll = 0.0, pitch = 0.0, yawrate = 0.0, zdistance = z)
            time.sleep(0.05)
            print('num0',i, 'tar',x,y,'cmd',roll_cmd,pitch_cmd )

    if sequence == sequence0:
        cf.commander.send_setpoint(roll = 0, pitch = 0, yawrate = 0, thrust = 0) # motor down, sit
        cf.commander.send_stop_setpoint()
        cf.commander.send_notify_setpoint_stop()
        global loop_run
        loop_run = False



def log_pos_callback(timestamp, data, logconf):
    print(timestamp)
    for name, value in data.items():
        print(f'{name}: {value:3.3f} ', end='')
    print()
    
    # position derived from stateEstimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']
    euler_estimate[0] = data['stateEstimate.roll']
    euler_estimate[1] = data['stateEstimate.pitch']
    euler_estimate[2] = data['stateEstimate.yaw']

    pos_x.append(position_estimate[0])
    pos_y.append(position_estimate[1])
    pos_z.append(position_estimate[2])
    roll.append(euler_estimate[0])
    pitch.append(euler_estimate[1])
    yaw.append(euler_estimate[2])
    cf_timestamp.append(timestamp)


def run_takeoff(scf):
    cf = scf.cf

    print("taking off")
    #ARM THROTTLE
    # cf.commander.send_setpoint(roll = 0, pitch = 0, yawrate = 0, thrust = 0)
    period = 20
    cf.commander.send_setpoint(roll = 0, pitch = 0, yawrate = 0, thrust = 0) # arm throttle

    for i in range(period):
        # cf.commander.send_position_setpoint(x, y, z, yaw)
        time.sleep(0.05)

        
        thrust_cmd = 42000 - 300
            

        # cf.commander.send_setpoint(roll = 0, pitch = 0, yawrate = 60, thrust = thrust_cmd)
        cf.commander.send_setpoint(roll = -0.15* euler_estimate[0], pitch = -0.15* euler_estimate[1], yawrate = 0, thrust = thrust_cmd) #compensate for error in rp
        
        # cf.commander.send_setpoint(roll = 0, pitch = 0, yawrate = 60, thrust = thrust_cmd) # circular move
    print("taked off")
    time.sleep(0.5)

### main starts here
if __name__ == '__main__':
    
    ## init driver
    cflib.crtp.init_drivers()
############ start drone loop
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:


#safety check, press enter after running
        input("Press Enter to continue log_pos...")

        scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(1)

        logconf = LogConfig(name='Position', period_in_ms=50)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        logconf.add_variable('stateEstimate.roll', 'float')
        logconf.add_variable('stateEstimate.pitch', 'float')
        logconf.add_variable('stateEstimate.yaw', 'float')
        scf.cf.log.add_config(logconf)
        #set karman filter value

        logconf.data_received_cb.add_callback(log_pos_callback)
        logconf.start()
        time.sleep(1)
        
        # run takeoff
        run_takeoff(scf)

        bluetoothSerial = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=1)
        time.sleep(0.5)
        loop_run = True
        while loop_run:
            #record position after taked off
            initial_x = position_estimate[0]
            initial_y = position_estimate[1]
            initial_z = position_estimate[2]
            initial_yaw = euler_estimate[0]
            current_signal = read_signal(bluetoothSerial)  # 模拟读取当前信号
            update_signal_history(signal_history, current_signal, max_history_length)

            stable_signal = find_stable_signal(signal_history, required_stable_count)
            if stable_signal and stable_signal in actions:
                # run_sequence(scf, actions[stable_signal], initial_x, initial_y, initial_z, initial_yaw) # 执行与稳定信号对应的行动
                time.sleep(1)
            else:
                # run_sequence(scf, sequence1, initial_x, initial_y, initial_z, initial_yaw)
                time.sleep(1)


        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        logconf.stop()






