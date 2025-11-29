from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import multitask, run_task, wait

# Set up all devices.
prime_hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
left = Motor(Port.F, Direction.COUNTERCLOCKWISE)
right = Motor(Port.E, Direction.CLOCKWISE)
drive_base = DriveBase(left, right, 62.4, 110)

error_prev = 0
error_curr = 0
error_sum = 0
error_threshold = 1
kp = 1
ki = 0.5
sum_cap = 30
kd = 0
dt = 0.010
min_rate = 22
curr = prime_hub.imu.heading()
proportional = 0
integral = 0
derivative = 0
dedt = 0
pid = 0
current_heading = 0

async def pid_turn_heading_async(desired, left, right, prime_hub):
    curr = prime_hub.imu.heading()
    error_prev = desired - curr
    error_curr = error_prev
    error_sum = 0
    dt = 0.010

    async def sub_pid():
        nonlocal error_prev, error_curr, error_sum
        curr = prime_hub.imu.heading()   
        error_curr = desired - curr
        proportional = kp * error_curr
        error_sum += error_curr * dt
        integral = ki * error_sum
        dedt = (error_curr - error_prev) / dt
        derivative = kd * dedt
        error_prev = error_curr
        pid = proportional + integral + derivative

        
        if error_sum > sum_cap:
            error_sum = sum_cap
        elif error_sum < -sum_cap:
            error_sum = -sum_cap

        print("P: ", proportional, " | I: ", integral, " | D: ", derivative, "Angle: ", curr)

        if pid > 100:
            pid = 100
        elif pid < min_rate and pid >= 0:
            pid = min_rate
        elif pid < -100:
            pid = -100
        elif pid > -min_rate and pid <= 0:
            pid = -min_rate

        left.dc(pid)
        right.dc(-pid)

    async def sub_dt():
        await wait(1)
    
    while not abs(error_curr) <= abs(error_threshold):
        await multitask(
            sub_pid(),
            sub_dt(),
        )
    
    left.hold()
    right.hold()

def _wrap_0_360(deg):
    if deg >= 360 or deg <= -360:
        deg = deg % 360
    return deg

async def pid_turn_incremental_async(desired, left_motor, right_motor, prime_hub):
    global current_heading
    target_heading = _wrap_0_360(current_heading + desired)
    await pid_turn_heading_async(target_heading, left_motor, right_motor, prime_hub)
    current_heading = target_heading    
