#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Script that holds useful macros for DCM walking tuning
'''

import numpy as np
from collections import defaultdict

# HIGH-LEVEL
# Walking
# foot swing trajectory
Ts     = 0.24   # desired stance phase duration [s]
Ts_min = 0.20   # minimum stance duration       [s]
Ts_max = 0.28   # maximum stance duration       [s]
T_buff = 0.05   # stop plan before T - T_buff   [s]

Txi = 0.00      # x stay before Txi
Txn = 0.05      # x go to nominal before Txn
Txf = 0.00      # x arrive before T - Txf

Tyi = 0.00      # y stay before Tyi
Tyn = 0.05      # y go to nominal before Tyn
Tyf = 0.00      # y arrive before T - Tyf

Tzm = 0.10      # desired swing apex time [s]
Tzf = 0.00      # z arrive before T - Tzf

zm_l = 0.035    # left  swing apex height [m]
zm_r = 0.035    # right swing apex height [m]

zf_l = -0.002   # left  swing final height [m]
zf_r = -0.002   # right swing final height [m]

hz = 0.365      # desired CoM height [m]

yaw_f_offset = 0.02  # foot yaw offset [rad]

# kinematic reachability [m]
lx  = 0.20      # max longitudinal step length
lyi = 0.04      # min lateral distance between feet
lyo = 0.20      # max lateral distance between feet

# velocity offset compensation [m]
bx_offset = +0.010  # set to negative if BRUCE tends to go forward
by_offset = +0.000  # set to negative if BRUCE tends to go left

# Stance
ka = -0.0       # x position of CoM from the center of foot, in scale of 1/2 foot length
                # ka = 1 puts CoM at the front tip of foot

# TOP-LEVEL
COM_POSITION_X     = 0
COM_POSITION_Y     = 1
COM_POSITION_Z     = 2

BODY_ORIENTATION_X = 3
BODY_ORIENTATION_Y = 4
BODY_ORIENTATION_Z = 5

COM_VELOCITY_X     = 6
COM_VELOCITY_Y     = 7
BODY_YAW_RATE      = 8

FOOT_YAW_RIGHT     = 9
FOOT_YAW_LEFT      = 10
FOOT_CLEARANCE     = 11

COOLING_SPEED      = 12
# Body COM / Body Orientation / COM Velocity / Body Yaw Rate / Swing Foot
PARAMETER_ID_LIST      = range(13)
PARAMETER_INCREMENT    = [ 0.05,  0.05,  0.002,       1,     1,     2,    0.01,  0.01,     1,       1,     1,  0.01,       1]
PARAMETER_DEFAULT      = [ 0.00,  0.00,  0.000,       0,     0,     0,     0.0,   0.0,     0,       0,     0,  0.05,       0]
PARAMETER_MAX          = [ 0.20,  0.50,  0.020,       8,    10,    30,    0.10,  0.10,    15,      10,    10,  0.20,       5]
PARAMETER_MIN          = [-0.20, -0.50, -0.200,      -8,   -10,   -20,   -0.10, -0.10,   -15,     -10,   -10,  0.03,       0]
# walk and balance can change without display.
PARAMETER_BUTTON_PLUS  = [  'g',   'j',    'l',     'y',   'i',   'p',     'w',   'a',   'q',     'x',   'v',   'm',     '=']
PARAMETER_BUTTON_MINUS = [  'f',   'h',    'k',     't',   'u',   'o',     's',   'd',   'e',     'z',   'c',   'n',     '-']
PARAMETER_TYPE         = ['len', 'len',  'len',   'ang', 'ang', 'ang',   'len', 'len', 'ang',   'ang', 'ang', 'len',   'len']
PARAMETER_RECOVER      = [  'y',   'y',    'y',     'y',   'y',   'y',     'y',   'y',   'y',     'y',   'y',   'y',     'n']

BALANCE = 0
WALK    = 1
ROCKING = 2
PARAMETER_MODE_LIST = {COM_POSITION_X:     [BALANCE],
                       COM_POSITION_Y:     [BALANCE],
                       COM_POSITION_Z:     [BALANCE, WALK],
                       BODY_ORIENTATION_X: [BALANCE],
                       BODY_ORIENTATION_Y: [BALANCE],
                       BODY_ORIENTATION_Z: [BALANCE],
                       COM_VELOCITY_X:     [WALK],
                       COM_VELOCITY_Y:     [WALK],
                       BODY_YAW_RATE:      [WALK],
                       FOOT_YAW_RIGHT:     [WALK],
                       FOOT_YAW_LEFT:      [WALK],
                       FOOT_CLEARANCE:     [WALK],
                       COOLING_SPEED:      [BALANCE, WALK]
                       }

# # wave trajectory
# arm_position_nominal = np.array([-0.7,  1.3,  2.0, 
#                                   0.7, -1.3, -2.0])
# arm_position_goal    = np.array([0.0, -1.2, 0.0,
#                                  0.0,  1.2, 0.0])
# arm_trajectory = defaultdict()

# for i in range(6):
#     arm_trajectory[i] = np.linspace(arm_position_nominal[i], arm_position_goal[i], 20, endpoint=True)

# traj_time = np.linspace(0, 2.75 * 2 * np.pi, 30)
# for tdx in traj_time:
#     arm_trajectory[1] = np.append(arm_trajectory[1], arm_position_goal[1] - 0.3 * np.sin(tdx))
#     arm_trajectory[4] = np.append(arm_trajectory[4], arm_position_goal[4] + 0.3 * np.sin(tdx))

#     for i in [0, 2, 3, 5]:
#         arm_trajectory[i] = np.append(arm_trajectory[i], arm_position_goal[i])

# for i in range(6):
#     arm_trajectory[i] = np.append(arm_trajectory[i], np.linspace(arm_trajectory[i][-1], arm_position_nominal[i], 20, endpoint=True))



arm_position_nominal = np.array([-0.7,  1.3,  2.0, 
                                  0.7, -1.3, -2.0])

pose1 = np.array([ 1.0,  0.1,  -1.5, # 0.2,  1.3,  1.2,
                    -1.5, 0, -1.3])
pose2 = np.array([ 1.0, 0.1, 1.3,
                   -1.0,  -0.1,  1.5])
pose3 = np.array([ -0.4,  0.2,  1.8,
                    0.4, -0.2, -1.8])

arm_trajectory = defaultdict(list)

def add_transition(src, dst, frames=15):
    return np.linspace(src, dst, frames, endpoint=True)

for i in range(6):
    arm_trajectory[i].extend(add_transition(arm_position_nominal[i], pose1[i]))

t1 = np.linspace(0, 1.5 * 2 * np.pi, 20)
amp1 = 0.2
for ti in t1:
    for i in range(6):
        if i in (1,4):
            arm_trajectory[i].append(pose1[i] + (i==1 and -1 or 1) * amp1 * np.sin(ti))
        else:
            arm_trajectory[i].append(pose1[i])
for i in range(6):
    arm_trajectory[i].extend(add_transition(arm_trajectory[i][-1], pose2[i]))

t2 = np.linspace(0, 1.5 * 2 * np.pi, 20)
for ti in t2:
    for i in range(6):
        if i in (1,4):
            arm_trajectory[i].append(pose2[i] + (i==4 and -1 or 1) * amp1 * np.sin(ti))
        else:
            arm_trajectory[i].append(pose2[i])
for i in range(6):
    arm_trajectory[i].extend(add_transition(arm_trajectory[i][-1], pose3[i]))

t3 = np.linspace(0, 2 * 2 * np.pi, 30)
amp3 = 0.15
for ti in t3:
    for i in range(6):
        if i in (1,4):
            arm_trajectory[i].append(pose3[i] + amp3 * np.sin(ti))
        else:
            arm_trajectory[i].append(pose3[i])


pose4 = np.array([
    0.8, -0.6, 0.9,    # 左臂前平举略上扬
   -0.6, 0.4, -0.7     # 右臂斜下张开
])

pose5 = np.array([
    0.6, -0.4, 1.4,    # 左臂向上摆，稍微向外
   -0.3, 0.7, -1.4     # 右臂较低、但抬起角度较大
])

for i in range(6):
    arm_trajectory[i].extend(add_transition(arm_trajectory[i][-1], pose4[i]))

t4 = np.linspace(0, 1.5 * 2 * np.pi, 20)
amp4 = 0.25
for ti in t4:
    for i in range(6):
        if i in (1, 4):
            direction = -1 if i == 1 else 1
            arm_trajectory[i].append(
                pose4[i] + direction * amp4 * np.sin(ti)
            )
        else:
            arm_trajectory[i].append(pose4[i])


for i in range(6):
    arm_trajectory[i].extend(
        add_transition(
            src=arm_trajectory[i][-1],
            dst=pose5[i],
            # frames=25
        )
    )


t5 = np.linspace(0, 2 * 2 * np.pi, 30)
amp5 = 0.18
for ti in t5:
    for i in range(6):
        if i == 1:
            arm_trajectory[i].append(
                pose5[i] + amp5 * (np.sin(ti) + 0.3 * np.cos(2*ti))
            )
        elif i == 4:
            arm_trajectory[i].append(
                pose5[i] + amp5 * np.sin(ti + np.pi/2)
            )
        else:
            arm_trajectory[i].append(pose5[i])

# for i in range(6):
#     arm_trajectory[i].extend(
#         add_transition(
#             src=arm_trajectory[i][-1],
#             dst=pose5[i],
#             # frames=20
#         )
#     )

            
for i in range(6):
    arm_trajectory[i].extend(add_transition(arm_trajectory[i][-1], arm_position_nominal[i], frames=20))

for i in range(6):
    arm_trajectory[i] = np.array(arm_trajectory[i])