import math

# Commands sent to the ESP32, one per line:
#   AIM_SET                          current heading becomes yaw 0
#   GOTO,yaw,pitch                   move to the start of the sweep (pump off)
#   TRAJ_BEGIN,n                     n LEG lines follow
#   LEG,i,time_ms,yaw,pitch          move to yaw/pitch in time_ms, both axes together
#   TRAJ_RUN                         go
# yaw is in degrees from the aim line, positive = left. pitch is degrees above horizontal.

MIN_HALF_SWEEP = 2.0
MAX_LEGS = 30

def serpentine_commands(fire_width_m, distance_m, beta_start, beta_end, yaw_rate, pitch_rate, margin=1.15):
    if fire_width_m <= 0 or distance_m <= 0 or yaw_rate <= 0 or pitch_rate <= 0:
        raise ValueError("width, distance and both rates must be positive")
    if beta_end < beta_start:
        raise ValueError("beta_end must be at least beta_start")

    half = math.degrees(math.atan(margin * fire_width_m / 2 / distance_m))
    half = max(half, MIN_HALF_SWEEP)

    leg_time = 2 * half / yaw_rate
    leg_ms = round(leg_time * 1000)

    yaw = half
    pitch = beta_start
    legs = []
    while True:
        yaw = -yaw
        pitch = min(pitch + pitch_rate * leg_time, beta_end)
        legs.append(f"LEG,{len(legs)},{leg_ms},{yaw:.2f},{pitch:.2f}")
        if pitch >= beta_end:
            break
        if len(legs) > MAX_LEGS:
            raise ValueError("too many legs, raise pitch_rate or lower yaw_rate")

    commands = ["AIM_SET", f"GOTO,{half:.2f},{beta_start:.2f}", f"TRAJ_BEGIN,{len(legs)}"]
    commands += legs
    commands.append("TRAJ_RUN")
    return commands