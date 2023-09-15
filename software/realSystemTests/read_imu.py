import utils
import asyncio
import moteus_pi3hat
import time
import numpy as np
import math
import moteus
import os
             
async def main():
    # Reset IMU
    moteus_pi3hat.Pi3HatRouter(mounting_deg={'roll': 0,
                                             'pitch': 0,
                                             'yaw': 0})  
    time.sleep(0.5)                                                   
    con = (0,90,180) # usb down
    #con = (-90,0,-90) # usb up
    # create a moteus_pi3hat.Pi3HatRouter for IMU reading
    pr = moteus_pi3hat.Pi3HatRouter(mounting_deg={'roll': con[0],
                                                  'pitch': con[1],
                                                  'yaw': con[2]})                                                
    time.sleep(4)                                               
    _, _, _, init_euler_xyz = await utils.read_imu_data(pr)               
    try:
        while True:               
            quat_wxyz, vel_xyz, acc_xyz, euler_xyz = await utils.read_imu_data(pr)               
            # quaternions
            w, x, y, z = quat_wxyz
            print(f'w={w:.4f},x={x:.4f},y={y:.4f},z={z:.4f}\n', end='\r')
            # angular verlocities (rad/s)
            wx, wy, wz = vel_xyz
            # print(f'wx={wx:.4f},wy={wy:.4f},wz={wz:.4f}\n')                    
            # angular accelerations (rad/s^2)
            acc_x, acc_y, acc_z = acc_xyz
            print(f'acc_x={acc_x:.4f},acc_y={acc_y:.4f},acc_z={acc_z:.4f}\n')                    
            # euler angles (rad)
            r, p, y = np.rad2deg(euler_xyz)
            print(f'roll={r:.4f},pitch={p:.4f},yaw={y:.4f}', end='\r')
            time.sleep(0.3)                    
    except BaseException as e:
        print(e)

if __name__ == '__main__': 
    asyncio.run(main())                                            
