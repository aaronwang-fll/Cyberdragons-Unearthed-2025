from pybricks.tools import wait
from pybricks.parameters import Stop

# ================== TUNING PARAMETERS ==================
p_turn_k_speed = 5.0     # proportional gain: deg/s per degree of error
min_speed = 50           # base speed to overcome friction
max_speed = 720          # maximum motor speed (deg/s)
tolerance = 1.0          # degrees: how close counts as "done"
dt_ms = 5                # loop sleep (ms)
stable_counts_needed = 2 # consecutive stable readings before stopping
max_ms = 8000            # safety timeout (ms) for a single turn

# Near-target easing
NEAR_DEG = 8.0           # below this error, use a gentler min speed
MIN_SPEED_NEAR = 40

# ================== STATE VARIABLES ==================
target_heading = 0
previous_heading = [0]
prev_heading_num = 0


# ================== HELPERS ==================
def _wrap_0_360(deg: float) -> float:
    """Normalize angle if beyond ±360, but preserve sign for small negatives."""
    if deg >= 360 or deg <= -360:
        deg = deg % 360
    return deg

def _wrapped_err(cur: float, desired: float) -> float:
    """Return signed smallest angular error in [-180, 180)."""
    return (desired - cur + 180) % 360 - 180

def _clamp(x: float, lo: float, hi: float) -> float:
    if x > hi:
        return hi
    if x < lo:
        return lo
    return x

def _stop_both(left_motor, right_motor):
    left_motor.hold()
    right_motor.hold()

# ================== CORE TURN LOGIC ==================
def _p_turn_heading_sync(desired, left_motor, right_motor, prime_hub):
    """Turn in place to an absolute heading using proportional *speed* control."""
    global prev_heading_num

    desired = _wrap_0_360(desired)
    stable_counts = 0
    elapsed = 0

    while True:
        cur = prime_hub.imu.heading()       # 0..359 from hub
        err = _wrapped_err(cur, desired)    # signed shortest difference (-180..180)

        if abs(err) <= tolerance:
            stable_counts += 1
            _stop_both(left_motor, right_motor)
            if stable_counts >= stable_counts_needed:
                break
        else:
            stable_counts = 0
            # Proportional speed control (deg/s)
            abs_err = abs(err)
            min_eff = MIN_SPEED_NEAR if abs_err < NEAR_DEG else min_speed
            speed = p_turn_k_speed * abs(err) + min_speed
            speed = _clamp(speed, -max_speed, max_speed)
            sgn = 1 if err > 0 else -1

            left_motor.run( sgn * speed)
            right_motor.run(-sgn * speed)

        wait(dt_ms)
        elapsed += dt_ms
        if elapsed >= max_ms:
            print("⚠️ Turn timeout, stopping for safety.")
            break

    # Stop cleanly
    _stop_both(left_motor, right_motor)

    previous_heading.append(desired)
    prev_heading_num += 1


async def _p_turn_heading_async(desired, left_motor, right_motor, prime_hub):
    """Async version using proportional *speed* control."""
    global prev_heading_num

    desired = _wrap_0_360(desired)
    stable_counts = 0
    elapsed = 0

    while True:
        cur = prime_hub.imu.heading()
        err = _wrapped_err(cur, desired)

        if abs(err) <= tolerance:
            stable_counts += 1
            _stop_both(left_motor, right_motor)
            if stable_counts >= stable_counts_needed:
                break
        else:
            stable_counts = 0
            abs_err = abs(err)
            min_eff = MIN_SPEED_NEAR if abs_err < NEAR_DEG else min_speed
            speed = p_turn_k_speed * abs_err + min_eff
            speed = _clamp(speed, -max_speed, max_speed)
            sgn = 1 if err > 0 else -1

            left_motor.run( sgn * speed)
            right_motor.run(-sgn * speed)

        await wait(dt_ms)
        elapsed += dt_ms
        if elapsed >= max_ms:
            print("⚠️ Turn timeout, stopping for safety.")
            break

    _stop_both(left_motor, right_motor)
    previous_heading.append(desired)
    prev_heading_num += 1


# ================== INCREMENTAL TURN HELPERS ==================
def p_turn_incremental_sync(turn_amount, left_motor, right_motor, prime_hub):
    """Turn by a relative amount (sync)."""
    global target_heading
    target_heading = _wrap_0_360(previous_heading[prev_heading_num] + turn_amount)
    _p_turn_heading_sync(target_heading, left_motor, right_motor, prime_hub)
    print("✅ Finished sync p_turn to", target_heading)


async def p_turn_incremental_async(turn_amount, left_motor, right_motor, prime_hub):
    """Turn by a relative amount (async)."""
    global target_heading
    target_heading = _wrap_0_360(previous_heading[prev_heading_num] + turn_amount)
    await _p_turn_heading_async(target_heading, left_motor, right_motor, prime_hub)
    print("✅ Finished async p_turn to", target_heading)


# ================== PIVOT VARIANTS (OPTIONAL) ==================
def _p_turn_pivot_sync(desired, pivot_left: bool, motor_left, motor_right, prime_hub):
    """Pivot around one wheel using proportional *speed* control."""
    desired = _wrap_0_360(desired)
    elapsed = 0

    while True:
        cur = prime_hub.imu.heading()
        err = _wrapped_err(cur, desired)
        if abs(err) <= tolerance:
            break

        abs_err = abs(err)
        min_eff = MIN_SPEED_NEAR if abs_err < NEAR_DEG else min_speed
        speed = p_turn_k_speed * abs_err + min_eff
        speed = _clamp(speed, -max_speed, max_speed)
        sgn = 1 if err > 0 else -1

        if pivot_left:
            motor_left.hold()
            motor_right.run(-sgn * speed)
        else:
            motor_right.hold()
            motor_left.run( sgn * speed)

        wait(dt_ms)
        elapsed += dt_ms
        if elapsed >= max_ms:
            print("⚠️ Pivot timeout, stopping for safety.")
            break

    motor_left.hold()
    motor_right.hold()


def p_turn_pivot_incremental_sync(turn_amount, pivot_left, motor_left, motor_right, prime_hub):
    global target_heading
    target_heading = _wrap_0_360(previous_heading[prev_heading_num] + turn_amount)
    _p_turn_pivot_sync(target_heading, pivot_left, motor_left, motor_right, prime_hub)


async def _p_turn_pivot_async(desired, pivot_left: bool, motor_left, motor_right, prime_hub):
    """Async pivot turn."""
    desired = _wrap_0_360(desired)
    elapsed = 0

    while True:
        cur = prime_hub.imu.heading()
        err = _wrapped_err(cur, desired)
        if abs(err) <= tolerance:
            break

        abs_err = abs(err)
        min_eff = MIN_SPEED_NEAR if abs_err < NEAR_DEG else min_speed
        speed = p_turn_k_speed * abs_err + min_eff
        speed = _clamp(speed, -max_speed, max_speed)
        sgn = 1 if err > 0 else -1

        if pivot_left:
            motor_left.hold()
            motor_right.run(-sgn * speed)
        else:
            motor_right.hold()
            motor_left.run( sgn * speed)

        await wait(dt_ms)
        elapsed += dt_ms
        if elapsed >= max_ms:
            print("⚠️ Pivot timeout, stopping for safety.")
            break

    motor_left.hold()
    motor_right.hold()


async def p_turn_pivot_incremental_async(turn_amount, pivot_left, motor_left, motor_right, prime_hub):
    global target_heading
    target_heading = _wrap_0_360(previous_heading[prev_heading_num] + turn_amount)
    await _p_turn_pivot_async(target_heading, pivot_left, motor_left, motor_right, prime_hub)
