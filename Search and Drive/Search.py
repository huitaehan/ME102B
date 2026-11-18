import time

# AIM_SET     current heading becomes yaw 0
# YAW,deg     turn to this yaw from the AIM_SET heading, positive = left
# CAM,deg     tilt the camera to this angle above horizontal
# DRIVE,m     drive straight forward this far
# Every command is answered with DONE or ERR,reason.
# YAW must be implemented as a chassis/base turn, not an independent turret axis.
# The integrated ESP32 currently rejects YAW: wheel hardware is not configured.

YAWS = [0, 45, -45]
PITCHES = [0, 45, -45]
DWELL_S = 5.0
SETTLE_S = 0.5
TURN_AFTER_SCAN = -90
DRIVE_M = 1.0


def run(send_and_wait, command):
    reply = send_and_wait(command)
    if not reply.startswith("DONE"):
        raise RuntimeError(f"{command} -> {reply}")


def fire_yaw_command(current_yaw_deg, relative_turn_deg):
    """Return YAW in the AIM_SET heading frame; never send a relative error as YAW.

    current_yaw_deg must come from measured base heading relative to AIM_SET.
    relative_turn_deg is Pi vision guidance, positive left. Reobserve after each
    completed turn. No heading feedback or drive hardware is implemented here.
    """
    import math
    if not all(math.isfinite(v) for v in (current_yaw_deg, relative_turn_deg)):
        raise ValueError('Heading and turn must be finite')
    target = (current_yaw_deg + relative_turn_deg + 180) % 360 - 180
    return f'YAW,{target:.3f}'


def search(send_and_wait, detect_fire, max_cycles=4):
    for cycle in range(max_cycles):
        run(send_and_wait, "AIM_SET")
        for pitch in PITCHES:
            run(send_and_wait, f"CAM,{pitch}")
            for yaw in YAWS:
                run(send_and_wait, f"YAW,{yaw}")
                time.sleep(SETTLE_S)
                fire = detect_fire(DWELL_S)
                if fire:
                    return {"yaw": yaw, "pitch": pitch, "detection": fire}
        run(send_and_wait, f"YAW,{TURN_AFTER_SCAN}")
        run(send_and_wait, f"DRIVE,{DRIVE_M}")
    return None
