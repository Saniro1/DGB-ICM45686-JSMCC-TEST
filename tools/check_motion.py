#!/usr/bin/env python3
"""Close JSM/other controller readers first. Install: python -m pip install hidapi."""
import argparse
import math
import statistics
import struct
import time


def main():
    import hid
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    candidates = [d for d in hid.enumerate(0x054c, 0x05c4)
                  if d.get('usage_page', 1) == 1 and d.get('usage', 5) == 5]
    if len(candidates) != 1:
        raise SystemExit(f'Found {len(candidates)} DS4 gamepad interfaces. Connect only the DGB device.')
    device = hid.device()
    device.open_path(candidates[0]['path'])
    arrivals, accel, gyro = [], [], []
    ticks = 0
    previous = None
    end = time.perf_counter() + args.seconds
    try:
        while time.perf_counter() < end:
            packet = bytes(device.read(64, 100))
            now = time.perf_counter()
            if len(packet) != 64 or packet[0] != 1:
                continue
            stamp = struct.unpack_from('<H', packet, 10)[0]
            if previous is not None:
                ticks += (stamp - previous) & 65535
            previous = stamp
            arrivals.append(now)
            gyro.append([x * 2000 / 32767 for x in struct.unpack_from('<hhh', packet, 13)])
            accel.append([x / 8192 for x in struct.unpack_from('<hhh', packet, 19)])
    finally:
        device.close()
    if len(arrivals) < 2:
        raise SystemExit('Insufficient reports. Check connection and close other controller readers.')
    count = len(arrivals) - 1
    print(f'Reports: {len(arrivals)}; host read rate: {count / (arrivals[-1] - arrivals[0]):.1f} Hz')
    if ticks:
        print(f'DS4 timestamp-derived rate: {count / (ticks * 16 / 3 / 1e6):.1f} Hz')
    for name, samples, unit in [('Accel', accel, 'g'), ('Gyro', gyro, 'deg/s')]:
        means = [statistics.mean(axis) for axis in zip(*samples)]
        print(f'{name} mean XYZ ({unit}): ' + ', '.join(f'{v:.4f}' for v in means))
        if name == 'Accel':
            print(f'Magnitude of mean acceleration: {math.sqrt(sum(v*v for v in means)):.4f} g')
    print('Keep device stationary for gravity/bias checks. Host reads can be buffered; this is not a USB bus trace.')


if __name__ == '__main__':
    main()
