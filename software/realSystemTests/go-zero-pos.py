import os
import asyncio
import moteus
import moteus_pi3hat
import utils
import numpy as np
import time

# from utils import go_to_pos
async def go_to_pos(servo, final_pos, rad_per_cycle=0.003, tau_limit = 4):
    (pos, _, _) = await utils.read_motor_data(servo)
    cmd_pos = pos
    sgn = -1 if pos > final_pos else 1
    try:
        while sgn * (final_pos - pos) > 0:
            cmd_pos += sgn * rad_per_cycle
            (pos,
            vel,
            tau) = await utils.send_rad_command(
                controller_obj=servo,
                pos=cmd_pos,
                vel=0,
                tau_limit=tau_limit)
    finally:
        await servo.set_stop()
async def main():
    print("Motor Enabled.")
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map={
            bus_number:[motor_id],
        }
    )
    # create a moteus.Controller object for interaction with controller
    servo = moteus.Controller(id=motor_id, transport=transport)
    await servo.set_stop()
    tau_limit = 3
    (pos, _, _) = await utils.read_motor_data(servo)
    cmd_pos = pos
    pos_cycle = 0.003
    print('init pos:', round(pos, 3))
    start_time = time.time()
    try:
        sgn = -1 if pos > final_pos else 1
        print('cw' if sgn == -1 else 'ccw')
        while sgn * (final_pos - pos) > 0:
            cmd_pos += sgn * pos_cycle
            (pos,
            vel,
            tau) = await utils.send_rad_command(
                controller_obj=servo,
                pos=cmd_pos,
                vel=0,
                tau_limit=tau_limit)        
    finally:
        await servo.set_stop()


async def test():
    await go_to_pos(
        servo=servo,
        final_pos=0,
        rad_per_cycle=0.003,
        tau_limit=4
    )


if __name__ == '__main__':
#    motor id and bus number mapping
    bus_number = 2
    motor_id = 9
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map={
            bus_number:[motor_id],
        }
    )
    servo = moteus.Controller(id=motor_id, transport=transport)
    while True:
        final_pos = np.deg2rad(float(input('Enter the final pos in degree:')))#np.deg2rad(0)        
        asyncio.run(main())
        asyncio.run(test())
    print('end of script')
