import lcm
import os
from Play.lcm_types.bruce_cmd_lidar import bruce_cmd_lidar_t
import time
import math

if __name__ == '__main__':
    # Control Frequency
    loop_freq = 10  # run     at 20 Hz
    loop_duration = 1 / loop_freq
    loop_iter = 0
    t0 = time.time()
    lidar_lcm = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
    lidar_cmd = bruce_cmd_lidar_t()
    while True:
        loop_start_time = time.time() - t0
        loop_target_time = loop_start_time + loop_duration
        lidar_cmd.com_z = 0.06 * math.sin(loop_iter * 0.3) - 0.05
        lidar_cmd.com_velocity[0] = 0.05
        lidar_cmd.ang_vel_yaw =15 * math.sin(loop_iter * 0.03)

        lidar_lcm.publish("bruce_cmd_lidar", lidar_cmd.encode())
        loop_iter+=1
        while time.time() - t0 < loop_target_time:
            pass