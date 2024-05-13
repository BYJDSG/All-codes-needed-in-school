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

import numpy as np

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')


# Change the sequence according to your setup
#             x    y    z
sequence = [
    (0.0, 0.0, 0.8),
    (-1.0, 0.0, 0.8),# position 1
    # (0.0, 0.0, 0.8),# position 1
    #(10, 2, 0.5),# position 2
    #(0, 0, 0.2),
]

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

def bluetooth_append_sequence(sequence):
    # print(sequence)
    sequence.append([0,0,0.9999999999])
    #bluetooth serial read
    # target_position = serial read data #[x, y, z] format
    # sequence.append(target_position)

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

        # bluetooth_append_sequence(sequence)
            
    cf.commander.send_setpoint(roll = 0, pitch = 0, yawrate = 0, thrust = 0) # motor down, sit
    cf.commander.send_stop_setpoint()
    cf.commander.send_notify_setpoint_stop()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing

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

        time.sleep(0.5)

        
        #record position after taked off
        initial_x = position_estimate[0]
        initial_y = position_estimate[1]
        initial_z = position_estimate[2]
        initial_yaw = euler_estimate[0]


        # run set points (sequence)
        run_sequence(scf, sequence, initial_x, initial_y, initial_z, initial_yaw)
        
        logconf.stop()
    


    ## plot xyz
    plt.figure(1)
    plt.plot(cf_timestamp, pos_x)
    plt.plot(cf_timestamp, pos_y)
    plt.plot(cf_timestamp, pos_z)
    plt.legend(["x", "y", "z"])
    plt.show()

    ## plot roll pitch yaw
    plt.figure(2)
    plt.plot(cf_timestamp, roll)
    plt.plot(cf_timestamp, pitch)
    plt.plot(cf_timestamp, yaw)
    
    plt.legend(["roll", "pitch", "yaw"])
    plt.show()