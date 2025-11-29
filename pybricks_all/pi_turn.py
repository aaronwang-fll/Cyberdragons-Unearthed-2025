from pybricks.tools import wait


# --- Tuning knobs (adjust as needed) ---
# PI gains
kp_turn = 1.0       # proportional gain (from program 2)
ki_turn = 0.4       # integral gain (from program 2)
sum_cap = 30.0      # cap for integral sum (anti-windup)

# base push (same idea as your p_turn_c, still passed in as c_parameter)
p_turn_c = 0    # default; you can override with c_parameter

min_rate = 30.0     # minimum motor.dc magnitude so robot doesn't stall
tolerance = 1.0     # degrees: how close counts as "done"
dt_ms = 5           # loop sleep/yield in milliseconds
dt_s = dt_ms / 1000 # seconds, for integral math

# --- State for incremental turns ---
target_heading = 0
current_heading = 0


# ----------------- helpers -----------------
def _wrap_0_360(deg: float) -> float:
    if deg >= 360 or deg <= -360:
        deg = deg % 360
    return deg

def _clamp_dc(x: float) -> float:
    """Clamp to motor.dc range [-100, 100]."""
    if x > 100:
        return 100
    if x < -100:
        return -100
    return x


# --------------- main turning logic (PI) ---------------
def _pi_turn_heading_sync(desired, left_motor, right_motor, prime_hub, c_parameter):
    """Turn in place to an absolute heading (degrees 0..359). Blocking version."""
    # Make sure desired is in 0..359 range, but we do NOT use wrapped error
    desired = _wrap_0_360(desired)

    # Use passed-in base push if you want to tune friction per-call
    base_push = c_parameter

    # Require a couple of consecutive "in tolerance" readings
    stable_counts_needed = 2
    stable_counts = 0

    # PI state
    err_prev = 0.0
    err_sum = 0.0

    while True:
        cur = prime_hub.imu.heading()   # 0..359 from hub
        # IMPORTANT: simple error (no shortest-path wrap), ok for up-to-360 turns
        err = desired - cur

        # Stop condition
        if abs(err) <= tolerance:
            stable_counts += 1
            if stable_counts >= stable_counts_needed:
                break
        else:
            stable_counts = 0

        # --- PI math ---
        p_term = kp_turn * err
        err_sum += err * dt_s

        # clamp integral sum
        if err_sum > sum_cap:
            err_sum = sum_cap
        elif err_sum < -sum_cap:
            err_sum = -sum_cap

        i_term = ki_turn * err_sum

        pid = p_term + i_term   # D = 0 for now

        # Optional: debug print
        # print("P:", p_term, "I:", i_term, "Angle:", cur)

        # --- Convert PID output to motor.dc ---
        # direction from sign of pid
        if pid >= 0:
            sgn = 1
        else:
            sgn = -1

        # magnitude from |pid| + base_push
        mag = abs(pid) + base_push

        # enforce minimum dc so we don't stall
        if mag < min_rate:
            mag = min_rate

        # clamp to [-100, 100]
        duty = _clamp_dc(mag)

        left_motor.dc( sgn * duty)
        right_motor.dc(-sgn * duty)

        wait(dt_ms)

    # Stop motors cleanly
    left_motor.dc(0)
    right_motor.dc(0)


async def _pi_turn_heading_async(desired, left_motor, right_motor, prime_hub, c_parameter):
    """Turn in place to an absolute heading (degrees 0..359). Async version."""
    desired = _wrap_0_360(desired)
    base_push = c_parameter

    stable_counts_needed = 2
    stable_counts = 0

    err_prev = 0.0
    err_sum = 0.0

    while True:
        cur = prime_hub.imu.heading()
        err = desired - cur

        if abs(err) <= tolerance:
            stable_counts += 1
            if stable_counts >= stable_counts_needed:
                break
        else:
            stable_counts = 0

        # --- PI math ---
        p_term = kp_turn * err
        err_sum += err * dt_s

        if err_sum > sum_cap:
            err_sum = sum_cap
        elif err_sum < -sum_cap:
            err_sum = -sum_cap

        i_term = ki_turn * err_sum

        pid = p_term + i_term

        # direction
        if pid >= 0:
            sgn = 1
        else:
            sgn = -1

        mag = abs(pid) + base_push

        if mag < min_rate:
            mag = min_rate

        duty = _clamp_dc(mag)

        left_motor.dc( sgn * duty)
        right_motor.dc(-sgn * duty)

        await wait(dt_ms)  # yield to scheduler

    left_motor.dc(0)
    right_motor.dc(0)


# --------------- incremental turns (same API) ---------------
def p_turn_incremental_sync(turn_amount, left_motor, right_motor, prime_hub, c_parameter):
    """Turn by a relative amount (degrees), blocking."""
    global current_heading
    # Add and wrap so targets are always 0..359
    target_heading = _wrap_0_360(current_heading + turn_amount)
    _pi_turn_heading_sync(target_heading, left_motor, right_motor, prime_hub, c_parameter)
    current_heading = target_heading


async def p_turn_incremental_async(turn_amount, left_motor, right_motor, prime_hub, c_parameter):
    """Turn by a relative amount (degrees), async."""
    global current_heading
    target_heading = _wrap_0_360(current_heading + turn_amount)
    await _pi_turn_heading_async(target_heading, left_motor, right_motor, prime_hub, c_parameter)
    current_heading = target_heading


# --------------- optional: pivot variants still P-only ---------------
def _p_turn_pivot_sync(desired, pivot_left: bool, motor_left, motor_right, prime_hub):
    """Pivot around one wheel to a desired heading. Not used by your script."""
    desired = _wrap_0_360(desired)
    while True:
        cur = prime_hub.imu.heading()
        err = desired - cur        # no wrap here either
        if abs(err) <= tolerance:
            break

        base = kp_turn * abs(err) + p_turn_c
        if base < min_rate:
            base = min_rate
        duty = _clamp_dc(base)
        sgn = 1 if err > 0 else -1

        if pivot_left:
            motor_left.dc(0)
            motor_right.dc(-sgn * duty)
        else:
            motor_right.dc(0)
            motor_left.dc( sgn * duty)
        # wait(dt_ms)  # blocking version, add if you want

    motor_left.dc(0)
    motor_right.dc(0)


async def _p_turn_pivot_async(desired, pivot_left: bool, motor_left, motor_right, prime_hub):
    """Pivot around one wheel to a desired heading. Async variant."""
    desired = _wrap_0_360(desired)
    while True:
        cur = prime_hub.imu.heading()
        err = desired - cur
        if abs(err) <= tolerance:
            break

        base = kp_turn * abs(err) + p_turn_c
        if base < min_rate:
            base = min_rate
        duty = _clamp_dc(base)
        sgn = 1 if err > 0 else -1

        if pivot_left:
            motor_left.dc(0)
            motor_right.dc(-sgn * duty)
        else:
            motor_right.dc(0)
            motor_left.dc( sgn * duty)

        await wait(dt_ms)

    motor_left.dc(0)
    motor_right.dc(0)


async def reset_current_heading():
    global current_heading
    current_heading = 0
