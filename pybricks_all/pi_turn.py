from pybricks.tools import wait

# =========================
#  TUNING CONSTANTS
# =========================

# Proportional gain: how hard to push based on current error
p_turn_k = 0.75

# Base push to overcome friction (you can tune this)
p_turn_c = 10.0

# Integral gain: how hard to push based on accumulated past error
i_turn_k = 0.35

# Limit for the integral term (anti-windup)
i_turn_max = 120.0

# How close (in degrees) counts as "good enough"
tolerance = 1      # LOWERED from 1.0

# Loop delay in milliseconds
dt_ms = 5


# =========================
#  STATE FOR INCREMENTAL TURNS
# =========================

# We track our own idea of heading (0..359) for incremental turns.
current_heading = 0


# =========================
#  HELPER FUNCTIONS
# =========================

def _wrap_0_360(deg):
    """Wrap any angle to the range [0, 360)."""
    if deg >= 360 or deg <= -360:
        deg = deg % 360
    return deg


def _wrapped_err(cur, desired):
    """
    Smallest signed difference between two angles, in degrees.
    Result is in [-180, 180).
    """
    return (desired - cur + 180) % 360 - 180


def _clamp_dc(x):
    """Clamp motor duty value to the valid range [-100, 100]."""
    if x > 100:
        return 100
    if x < -100:
        return -100
    return x


# =========================
#  CORE HEADING TURN (SYNC)
# =========================

def _p_turn_heading_sync(desired, left_motor, right_motor, prime_hub, c_parameter):
    """
    Turn in place to an absolute heading (degrees 0..359).
    BLOCKING (synchronous) version.
    """
    desired = _wrap_0_360(desired)

    # Use the given c_parameter for base friction push
    p_turn_c_local = 18

    # Require a few stable readings inside tolerance before we stop
    stable_counts_needed = 2
    stable_counts = 0

    # Integral term (memory of past error)
    integral = 0.0

    while True:
        # 1. Read current heading from hub
        cur = prime_hub.imu.heading()      # 0..359

        # 2. Compute smallest signed error (-180..180)
        err = _wrapped_err(cur, desired)

        # 3. Check if we are within tolerance
        if abs(err) <= tolerance:
            stable_counts += 1
            if stable_counts >= stable_counts_needed:
                break    # We are close enough, long enough → stop
        else:
            stable_counts = 0

        # ---------- PI CONTROL LOGIC ----------

        # Convert dt to seconds for the integral math
        dt_s = dt_ms / 1000.0

        # Update integral with signed error
        integral += err * dt_s

        # Anti-windup: keep integral from growing too big
        if integral > i_turn_max:
            integral = i_turn_max
        elif integral < -i_turn_max:
            integral = -i_turn_max

        # PI controller output (signed control value)
        control = p_turn_k * err + i_turn_k * integral

        # Use the sign of control to decide direction
        if control >= 0:
            sgn = 1
        else:
            sgn = -1

        # Magnitude of drive: |control| + base friction push
        base = abs(control) + p_turn_c_local

        # Clamp to [-100, 100] after we know the magnitude
        duty = _clamp_dc(base)

        # Apply to motors: spin in place
        left_motor.dc(_clamp_dc( sgn * duty))
        right_motor.dc(_clamp_dc(-sgn * duty))

        # Small pause to control loop speed
        wait(dt_ms)

    # Stop motors cleanly when done
    left_motor.dc(0)
    right_motor.dc(0)


# =========================
#  CORE HEADING TURN (ASYNC)
# =========================

async def _p_turn_heading_async(desired, left_motor, right_motor, prime_hub, c_parameter):
    """
    Turn in place to an absolute heading (degrees 0..359).
    ASYNC (awaitable) version.
    """
    desired = _wrap_0_360(desired)

    p_turn_c_local = 18
    stable_counts_needed = 2
    stable_counts = 0
    integral = 0.0

    while True:
        cur = prime_hub.imu.heading()
        err = _wrapped_err(cur, desired)

        if abs(err) <= tolerance:
            stable_counts += 1
            if stable_counts >= stable_counts_needed:
                break
        else:
            stable_counts = 0

        dt_s = dt_ms / 1000.0
        integral += err * dt_s

        if integral > i_turn_max:
            integral = i_turn_max
        elif integral < -i_turn_max:
            integral = -i_turn_max

        control = p_turn_k * err + i_turn_k * integral

        if control >= 0:
            sgn = 1
        else:
            sgn = -1

        base = abs(control) + p_turn_c_local
        duty = _clamp_dc(base)

        left_motor.dc(_clamp_dc( sgn * duty))
        right_motor.dc(_clamp_dc(-sgn * duty))

        await wait(dt_ms)

    left_motor.dc(0)
    right_motor.dc(0)


# =========================
#  INCREMENTAL TURNS
# =========================

def p_turn_incremental_sync(turn_amount, left_motor, right_motor, prime_hub, c_parameter):
    """
    Turn by a relative amount (degrees).
    Example: +90 means "turn 90° to the left" from current_heading.
    BLOCKING version.
    """
    global current_heading

    # Compute new target heading
    target_heading = _wrap_0_360(current_heading + turn_amount)

    # Do the turn to that absolute heading
    _p_turn_heading_sync(target_heading, left_motor, right_motor, prime_hub, c_parameter)

    # Update our internal heading tracker
    current_heading = target_heading


async def p_turn_incremental_async(turn_amount, left_motor, right_motor, prime_hub, c_parameter):
    """
    Turn by a relative amount (degrees).
    ASYNC version.
    """
    global current_heading

    target_heading = _wrap_0_360(current_heading + turn_amount)
    await _p_turn_heading_async(target_heading, left_motor, right_motor, prime_hub, c_parameter)
    current_heading = target_heading


# =========================
#  RESET HEADING TRACKER
# =========================

async def reset_current_heading(prime_hub=None):
    """
    Reset the internal current_heading to 0.
    If prime_hub is given and you calibrated the IMU so that
    the real heading is also 0, then everything lines up again.
    """
    global current_heading
    current_heading = 0

    # Optional: if you like, you could also check prime_hub.imu.heading()
    # or re-calibrate here.

