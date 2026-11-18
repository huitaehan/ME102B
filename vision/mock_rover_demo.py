"""Scripted Pi -> ESP32 mock turn -> projectile solve -> simulated shot test.

No camera or motors needed. Upload FireTurret with DEMO_MODE=true first.
The synthetic target moves in the virtual image as BASE_MOCK heading changes.
"""

import argparse
import math
import time
from types import SimpleNamespace

if __package__:
    from .fire_serial_bridge import mock_target_message, solve_request, start_mock_session
else:
    from fire_serial_bridge import mock_target_message, solve_request, start_mock_session


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--yaw-deg', type=float, default=20.0)
    parser.add_argument('--distance-m', type=float, default=0.8)
    parser.add_argument('--jet-speed-m-s', type=float, default=5.0)
    args = parser.parse_args()
    if not math.isfinite(args.yaw_deg) or abs(args.yaw_deg) > 90:
        parser.error('yaw-deg must be finite and between -90 and 90')
    if not math.isfinite(args.distance_m) or not 0.1 <= args.distance_m <= 2.0:
        parser.error('distance-m must be within the firmware shooting range 0.1 to 2 metres')
    if not math.isfinite(args.jet_speed_m_s) or args.jet_speed_m_s <= 0:
        parser.error('jet-speed-m-s must be finite and positive')
    solver_args = SimpleNamespace(
        jet_speed_m_s=args.jet_speed_m_s, target_height_m=0.2,
        nozzle_length_m=0.1, turret_x_m=0.0, turret_y_m=0.0, base_height_m=0.2)

    import serial
    link = serial.Serial(args.port, 115200, timeout=0.1, write_timeout=1)
    try:
        time.sleep(2)
        start_mock_session(link)
        print('MOCK ONLY: example geometry/speed; heading and encoder readings are simulated.', flush=True)
        heading = 0.0
        sequence = 0
        next_frame = 0.0
        deadline = time.monotonic() + 40
        shot_seen = False
        clear_target = False
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_frame:
                sequence += 1
                guidance = None if clear_target else SimpleNamespace(
                    distance_m=args.distance_m, confidence=0.95,
                    yaw_error_deg=args.yaw_deg-heading)
                link.write(mock_target_message(sequence, guidance).encode('ascii'))
                next_frame = now + 0.1
            line = link.readline().decode('ascii', errors='replace').strip()
            if not line:
                continue
            print(line, flush=True)
            if line.startswith('BASE_MOCK,'):
                heading = float(line.split(',')[1])
            reply = solve_request(line, solver_args)
            if reply is not None:
                print('Pi -> ' + reply.strip(), flush=True)
                link.write(reply.encode('ascii'))
            if line == 'STATE,6':
                shot_seen = True
            if line == 'STATE,7' and shot_seen:
                clear_target = True
            if line == 'STATE,2' and clear_target:
                print('PASS: align -> Pi projectile solve -> simulated shot -> fire check -> search', flush=True)
                return
            if line == 'STATE,8' or line.startswith('ERR,'):
                raise RuntimeError('ESP32 rejected/faulted the mock run: ' + line)
        raise RuntimeError('Mock run timed out')
    except KeyboardInterrupt:
        print('Mock interrupted')
    finally:
        try:
            link.write(b'STOP\n')
            link.flush()
        finally:
            link.close()


if __name__ == '__main__':
    main()
