# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Simple example that connects to one crazyflie, sets the initial position/yaw
and flies a trajectory.

The initial pose (x, y, z, yaw) is configured in a number of variables and
the trajectory is flown relative to this position, using the initial yaw.

This example is intended to work with any absolute positioning system.
It aims at documenting how to take off with the Crazyflie in an orientation
that is different from the standard positive X orientation and how to set the
initial position of the kalman estimator.
"""
import math
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander

import matplotlib.pyplot as plt
import cv2


import serial



# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')


# Change the sequence according to your setup
#             x    y    z
# sequence = [
#     (0.0, 0.0, 0.5),
#     #(0, 2, 0.5),# position 1
#     #(10, 2, 0.5),# position 2
#     #(0, 0, 0.2),
# ]

global position_estimate
global euler_estimate
global data
global bluetoothSerial
position_estimate = [0, 0, 0]
euler_estimate = [0, 0, 0]
data = ''
bluetoothSerial = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=1)

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

def get_glove_info():
    pass

def run_sequence(scf, base_x, base_y, base_z, yaw):
    cf = scf.cf
    try:
        while True:
            if bluetoothSerial.inWaiting() > 0:
                data = bluetoothSerial.readline().decode('utf-8').rstrip()
                position = analyzedata(data)
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
                    # x_for_first_command = position_estimate[0]
                    # y_for_first_command = position_estimate[1]
                    print(i)
                    
            cf.commander.send_zdistance_setpoint(roll = -0.06*euler_estimate[0], pitch = -0.06*euler_estimate[1], yawrate = - euler_estimate[2], zdistance = 0.5)
            time.sleep(1)
            # cf.commander.send_stop_setpoint()
            # # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
            # cf.commander.send_notify_setpoint_stop()

            # # Make sure that the last packet leaves before the link is closed
            # # since the message queue is not flushed before closing
            # time.sleep(1)
            # time.sleep(0.1)
    except KeyboardInterrupt:
        bluetoothSerial.close()  # 确保端口被正确关闭



def run_takeoff(scf):
    cf = scf.cf

    print("taking off")
    #ARM THROTTLE
    # cf.commander.send_setpoint(roll = 0, pitch = 0, yawrate = 0, thrust = 0)
    period = 300
    for i in range(period):
        # cf.commander.send_position_setpoint(x, y, z, yaw)
        # time.sleep(0.1)

        if i < 4: 
            thrust_cmd = 39500 + 500 + 1000
        elif i < period/2: 
            thrust_cmd = 39500 + 230 + 1000
            
        elif i < period: 
            thrust_cmd = 39000 + 0 + 1000

        # cf.commander.send_setpoint(roll = 0, pitch = 0, yawrate = 60, thrust = thrust_cmd)
        cf.commander.send_setpoint(roll = -0.05* euler_estimate[0], pitch = -0.05* euler_estimate[1], yawrate = 0, thrust = thrust_cmd) #compensate for error in rp
        
        # cf.commander.send_setpoint(roll = 0, pitch = 0, yawrate = 60, thrust = thrust_cmd) # circular move
    print("taked off")

def log_pos_callback(timestamp, data, logconf):
    # print(data)
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
    # print(timestamp, position_estimate, euler_estimate)

    pos_x.append(position_estimate[0])
    pos_y.append(position_estimate[1])
    pos_z.append(position_estimate[2])


    
    roll.append(euler_estimate[0])
    pitch.append(euler_estimate[1])
    yaw.append(euler_estimate[2])

    cf_timestamp.append(timestamp)
    
DEFAULT_HEIGHT = 0.5



# def move_linear_simple(scf):
#     with MotionCommander(scf) as mc:
#         mc.start_circle_left(radius_m = 0.2, velocity=0.2)
#         time.sleep(4)
#         mc.stop()
        

if __name__ == '__main__':
    cflib.crtp.init_drivers()



    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        input("Press Enter to continue log_pos...")


        scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

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

        initial_x = position_estimate[0]
        initial_y = position_estimate[1]
        initial_z = position_estimate[2]
        initial_yaw = euler_estimate[0]


        # run set points (sequence)
        run_sequence(scf, initial_x, initial_y, initial_z, initial_yaw)
        # move_linear_simple(scf) # MotionCommand  need EKF prob
        # time.sleep(5)

        logconf.stop()
    i = 0



    plt.figure(1)
    plt.plot(cf_timestamp, pos_x)
    plt.plot(cf_timestamp, pos_y)
    plt.plot(cf_timestamp, pos_z)
    plt.show()

    
    plt.figure(2)
    plt.plot(cf_timestamp, roll)
    plt.plot(cf_timestamp, pitch)
    plt.plot(cf_timestamp, yaw)
    plt.show()