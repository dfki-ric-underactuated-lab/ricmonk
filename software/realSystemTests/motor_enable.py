import asyncio
import moteus
import moteus_pi3hat
## documentation https://github.com/mjbots/moteus/tree/main/lib/python
import time
import os
from utils import zero_offset

if __name__ == '__main__':
    bus_number = [2]
    motor_id = [8, 9]
    zero_offset(bus_number[0], motor_id[0])
    zero_offset(bus_number[0], motor_id[1])  
    
    #motor_id = [14]
    #zero_offset(bus_number[0], motor_id[0])
   # while True:
   #     pass
