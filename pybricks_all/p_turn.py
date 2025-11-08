from pybricks.tools import wait

# --- Tuning knobs (adjust as needed) ---
p_turn_k = 0.4      # proportional gain
p_turn_c = 22       # base push to overcome friction (added magnitude)
tolerance = 1.0     # degrees: how close counts as "done"
dt_ms = 5          # loop sleep/yield in milliseconds

# --- State for incremental turns ---
target_heading = 0
previous_heading = [0]   # start assuming we began at 0°
prev_heading_num = 0


# ----------------- helpers -----------------
def _wrap_0_360(deg: float) -> float:
    return deg % 360

def _wrapped_err(cur: float, desired: float) -> float:
    """Return signed smallest error in degrees, in [-180, 180)."""
    return (desired - cur + 180) % 360 - 180

def _clamp_dc(x: float) -> float:
    """Clamp to motor.dc range [-100, 100]."""
    if x > 100:
        return 100
    if x < -100:
        return -100
    return x


# --------------- main turning logic ---------------
def _p_turn_heading_sync(desired, left_motor, right_motor, prime_hub):
    """Turn in place to an absolute heading (degrees 0..359). Awaitable."""
    global prev_heading_num

    desired = _wrap_0_360(desired)

    # Optional: require a couple of consecutive "in tolerance" readings
    stable_counts_needed = 2
    stable_counts = 0

    while True:
        cur = prime_hub.imu.heading()              # 0..359 from hub
        err = _wrapped_err(cur, desired)           # -180..+180 (shortest way)

        if abs(err) <= tolerance:
            stable_counts += 1
            if stable_counts >= stable_counts_needed:
                break
        else:
            stable_counts = 0

        # Duty = sign(err) * (k*|err| + c)
        base = p_turn_k * abs(err) + p_turn_c
        duty = _clamp_dc(base)
        sgn = 1 if err > 0 else -1

        left_motor.dc(_clamp_dc( sgn * duty))
        right_motor.dc(_clamp_dc(-sgn * duty))

        # await wait(dt_ms)  # yield to scheduler

    # Stop motors cleanly
    left_motor.dc(0)
    right_motor.dc(0)

    previous_heading.append(desired)
    prev_heading_num += 1

async def _p_turn_heading_async(desired, left_motor, right_motor, prime_hub):
    """Turn in place to an absolute heading (degrees 0..359). Awaitable."""
    global prev_heading_num

    desired = _wrap_0_360(desired)

    # Optional: require a couple of consecutive "in tolerance" readings
    stable_counts_needed = 2
    stable_counts = 0

    while True:
        cur = prime_hub.imu.heading()              # 0..359 from hub
        err = _wrapped_err(cur, desired)           # -180..+180 (shortest way)

        if abs(err) <= tolerance:
            stable_counts += 1
            if stable_counts >= stable_counts_needed:
                break
        else:
            stable_counts = 0

        # Duty = sign(err) * (k*|err| + c)
        base = p_turn_k * abs(err) + p_turn_c
        duty = _clamp_dc(base)
        sgn = 1 if err > 0 else -1

        left_motor.dc(_clamp_dc( sgn * duty))
        right_motor.dc(_clamp_dc(-sgn * duty))

        await wait(dt_ms)  # yield to scheduler

    # Stop motors cleanly
    left_motor.dc(0)
    right_motor.dc(0)

    previous_heading.append(desired)
    prev_heading_num += 1


def p_turn_incremental_sync(turn_amount, left_motor, right_motor, prime_hub):
    """Turn by a relative amount (degrees), awaitable."""
    global target_heading
    # Add and wrap so targets are always 0..359
    target_heading = _wrap_0_360(previous_heading[prev_heading_num] + turn_amount)
    #await p_turn__heading_(target_heading, left_motor, right_motor, prime_hub)
    _p_turn_heading_sync(target_heading, left_motor, right_motor, prime_hub)
    print("finished pturn")

async def p_turn_incremental_async(turn_amount, left_motor, right_motor, prime_hub):
    """Turn by a relative amount (degrees), awaitable."""
    global target_heading
    # Add and wrap so targets are always 0..359
    target_heading = _wrap_0_360(previous_heading[prev_heading_num] + turn_amount)
    await _p_turn_heading_async(target_heading, left_motor, right_motor, prime_hub)
    print("finished pturn")


# (optional) pivot variants if you ever want one-wheel turns
def _p_turn_pivot_sync(desired, pivot_left: bool, motor_left, motor_right, prime_hub):
    """Pivot around one wheel to a desired heading. Not used by your script."""
    desired = _wrap_0_360(desired)
    while True:
        cur = prime_hub.imu.heading()
        err = _wrapped_err(cur, desired)
        if abs(err) <= tolerance:
            break
        base = p_turn_k * abs(err) + p_turn_c
        duty = _clamp_dc(base)
        sgn = 1 if err > 0 else -1
        if pivot_left:
            motor_left.dc(0)
            motor_right.dc(_clamp_dc(-sgn * duty))
        else:
            motor_right.dc(0)
            motor_left.dc(_clamp_dc( sgn * duty))
        # await wait(dt_ms)
    motor_left.dc(0)
    motor_right.dc(0)


def p_turn_pivot_incremental_sync(turn_amount, pivot_left, motor_left, motor_right, prime_hub):
    global target_heading
    target_heading = _wrap_0_360(previous_heading[prev_heading_num] + turn_amount)
    _p_turn_pivot_sync(target_heading, pivot_left, motor_left, motor_right, prime_hub)

async def _p_turn_pivot_async(desired, pivot_left: bool, motor_left, motor_right, prime_hub):
    """Pivot around one wheel to a desired heading. Not used by your script."""
    desired = _wrap_0_360(desired)
    while True:
        cur = prime_hub.imu.heading()
        err = _wrapped_err(cur, desired)
        if abs(err) <= tolerance:
            break
        base = p_turn_k * abs(err) + p_turn_c
        duty = _clamp_dc(base)
        sgn = 1 if err > 0 else -1
        if pivot_left:
            motor_left.dc(0)
            motor_right.dc(_clamp_dc(-sgn * duty))
        else:
            motor_right.dc(0)
            motor_left.dc(_clamp_dc( sgn * duty))
        await wait(dt_ms)
    motor_left.dc(0)
    motor_right.dc(0)


async def p_turn_pivot_incremental_async(turn_amount, pivot_left, motor_left, motor_right, prime_hub):
    global target_heading
    target_heading = _wrap_0_360(previous_heading[prev_heading_num] + turn_amount)
    await _p_turn_pivot_async(target_heading, pivot_left, motor_left, motor_right, prime_hub)