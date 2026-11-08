"""Pi entry point for vision, serial observations, and projectile solving.

Run from the repository root. Arguments are identical to fire_serial_bridge.py.
Search/driving and serpentine execution require additional hardware services;
they are not launched concurrently against the same serial port.
"""

from vision.fire_serial_bridge import main


if __name__ == '__main__':
    main()

