from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Direction, Port, Stop
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import run_task, wait

from p_turn import p_turn_incremental_async

# Set up all devices.
prime_hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
right_light = ColorSensor(Port.A)
left_light = ColorSensor(Port.B)
left = Motor(Port.F, Direction.COUNTERCLOCKWISE)
right = Motor(Port.E, Direction.CLOCKWISE)
drive_base = DriveBase(left, right, 62.4, 110)
left_attachment = Motor(Port.D, Direction.CLOCKWISE)
right_attachment = Motor(Port.C, Direction.COUNTERCLOCKWISE)

while 1 == 1:
    left.dc(80)
    right.dc(80)