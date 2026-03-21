from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import run_task, wait

from pi_turn import p_turn_incremental_sync, p_turn_incremental_async


async def _s_turn(radius_one, angle_one, radius_two, angle_two, speed, left, right, primehub, driverbaser):

    driverbaser.settings(straight_speed=speed)
    if radius_one >= 1000:
        await driverbaser.straight(angle_one, then=stop.NONE)
    else:
        await driverbaser.arc(radius_one, angle=angle_one, then=Stop.NONE)

    if radius_two >= 1000:
        await driverbaser.straight(angle_two, then=Stop.COAST)
    else:
        await driverbaser.arc(radius_two, angle=angle_two, then=Stop.COAST)