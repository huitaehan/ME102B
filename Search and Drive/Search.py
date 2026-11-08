import time

# AIM_SET     current heading becomes yaw 0
# YAW,deg     turn to this yaw from the AIM_SET heading, positive = left
# CAM,deg     tilt the camera to this angle above horizontal
# DRIVE,m     drive straight forward this far
# Every command is answered with DONE or ERR,reason.

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