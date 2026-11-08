# HOME,axis,direction,speed_dps,max_travel_deg,switch_angle_deg
# The ESP32 moves the axis in that direction until its limit switch trips, stops,
# sets its angle to switch_angle_deg, then replies DONE,axis or ERR,axis,reason.

TURRET_DIRECTION = 1
TURRET_SWITCH_ANGLE = 90.0
CAM_DIRECTION = -1
CAM_SWITCH_ANGLE = -90.0

HOME_SPEED_DPS = 10.0
MAX_TRAVEL_DEG = 120.0


def home_command(axis, direction, speed_dps, max_travel_deg, switch_angle_deg):
    if axis not in ("TURRET", "CAM"):
        raise ValueError("axis must be TURRET or CAM")
    if direction not in (1, -1):
        raise ValueError("direction must be 1 or -1")
    if speed_dps <= 0 or max_travel_deg <= 0:
        raise ValueError("speed and max travel must be positive")
    return f"HOME,{axis},{direction},{speed_dps:.1f},{max_travel_deg:.1f},{switch_angle_deg:.1f}"


def homing_commands():
    turret = home_command("TURRET", TURRET_DIRECTION, HOME_SPEED_DPS, MAX_TRAVEL_DEG, TURRET_SWITCH_ANGLE)
    cam = home_command("CAM", CAM_DIRECTION, HOME_SPEED_DPS, MAX_TRAVEL_DEG, CAM_SWITCH_ANGLE)
    return [turret, cam]


def home_all(send_and_wait):
    for command in homing_commands():
        reply = send_and_wait(command)
        if not reply.startswith("DONE"):
            raise RuntimeError(f"homing failed: {command} -> {reply}")