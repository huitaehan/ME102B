"""IMX500 accelerator + D435i depth-only preview; never falls back to CPU YOLO."""

from realsense_fire_preview import main


if __name__ == '__main__':
    main(require_dual=True)
