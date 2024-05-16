import serial
import time

signal_history = []
loop_run = False
max_history_length = 10
required_stable_count = 6

def read_signal(bluetoothSerial):
    data_from_com = []
    # 忽略第一个数据包
    initial_line = bluetoothSerial.readline().decode('utf-8').rstrip()
    print("忽略第一个数据包:", initial_line)
    if bluetoothSerial.inWaiting() > 0:
        data_from_com = bluetoothSerial.readline().decode('utf-8').rstrip().split(',')
    time.sleep(0.1)
    mode = int(data_from_com[0])
    X_acc = 1 if float(data_from_com[1]) > 0.05  else -1 if float(data_from_com[1]) < -0.05 else 0
    Y_acc = 1 if float(data_from_com[2]) > 0.05  else -1 if float(data_from_com[2]) < -0.05 else 0
    Z_acc = 1 if float(data_from_com[3]) > 0.05  else -1 if float(data_from_com[3]) < -0.05 else 0
    gloveRoll = 1 if float(data_from_com[4]) > 100 else 0
    glovePitch = 1 if float(data_from_com[5]) > 100 else 0
    gloveYaw = 1 if float(data_from_com[6]) > 100 else 0
    return (mode, X_acc, Y_acc, Z_acc, gloveRoll, glovePitch, gloveYaw)

def update_signal_history(signal_history, new_signal, max_history_length):
    # 在列表的开始位置插入新的信号
    signal_history.insert(0, tuple(new_signal))
    
    # 如果历史记录的长度超过了设定的最大长度，则移除最旧的信号（列表末尾的元素）
    if len(signal_history) > max_history_length:
        signal_history.pop()

def find_stable_signal(signal_history, required_stable_count):
    if len(signal_history) < required_stable_count:
        return None

    # 检查每个可能的信号模式是否在历史中稳定
    for signal_pattern in set(signal_history):
        if signal_history.count(signal_pattern) >= required_stable_count:
            return signal_pattern
    return None

### main starts here
if __name__ == '__main__':
    bluetoothSerial = serial.Serial('COM6', baudrate=9600, timeout=1)
    time.sleep(0.5)
    loop_run = True
    while loop_run:
        
        current_signal = read_signal(bluetoothSerial)  # 模拟读取当前信号
        print(current_signal)
        update_signal_history(signal_history, current_signal, max_history_length)
        stable_signal = find_stable_signal(signal_history, required_stable_count)
        