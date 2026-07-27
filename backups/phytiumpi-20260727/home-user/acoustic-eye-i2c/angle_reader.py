#!/usr/bin/env python3
import argparse
import fcntl
import json
import os
import sys
import time

I2C_SLAVE = 0x0703
PACKET_SIZE = 8


def decode_packet(packet: bytes) -> dict:
    if len(packet) != PACKET_SIZE:
        raise ValueError(f"short packet: {len(packet)}")
    if packet[0] != 0xA5 or packet[1] != 0x01:
        raise ValueError(f"bad header: {packet.hex()}")
    checksum = 0
    for value in packet[:7]:
        checksum ^= value
    if checksum != packet[7]:
        raise ValueError(f"bad checksum: {packet.hex()}")

    status = packet[2]
    angle_tenth = packet[3] | (packet[4] << 8)
    sequence = packet[5] | (packet[6] << 8)
    return {
        "angle_deg": angle_tenth / 10.0,
        "stable": bool(status & 0x01),
        "current_valid": bool(status & 0x02),
        "device_ok": bool(status & 0x04),
        "sequence": sequence,
        "status": status,
        "timestamp": time.time(),
    }


def write_state(path: str, state: dict) -> None:
    temporary = f"{path}.tmp"
    with open(temporary, "w", encoding="utf-8") as output:
        json.dump(state, output, ensure_ascii=False, separators=(",", ":"))
        output.write("\n")
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser(description="Read AcousticEye angle from STM32 I2C slave")
    parser.add_argument("--device", default="/dev/i2c-3")
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x42)
    parser.add_argument("--output", default="/run/acoustic-eye/angle.json")
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    descriptor = os.open(args.device, os.O_RDONLY)
    try:
        fcntl.ioctl(descriptor, I2C_SLAVE, args.address)
        while True:
            try:
                state = decode_packet(os.read(descriptor, PACKET_SIZE))
                write_state(args.output, state)
                if args.verbose:
                    print(json.dumps(state, ensure_ascii=False), flush=True)
            except (OSError, ValueError) as error:
                print(f"angle read failed: {error}", file=sys.stderr, flush=True)
            time.sleep(args.interval)
    finally:
        os.close(descriptor)


if __name__ == "__main__":
    raise SystemExit(main())
