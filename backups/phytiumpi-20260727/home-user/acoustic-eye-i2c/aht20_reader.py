#!/usr/bin/env python3
import argparse
import fcntl
import json
import os
import sys
import time

I2C_SLAVE = 0x0703
AHT20_ADDRESS = 0x38
MEASURE_COMMAND = b"\xAC\x33\x00"
INITIALIZE_COMMAND = b"\xBE\x08\x00"


def crc8(data: bytes) -> int:
    value = 0xFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x31) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def read_exact(descriptor: int, count: int) -> bytes:
    data = os.read(descriptor, count)
    if len(data) != count:
        raise OSError(f"short AHT20 response: {len(data)} bytes")
    return data


def initialize(descriptor: int) -> None:
    os.write(descriptor, b"\x71")
    status = read_exact(descriptor, 1)[0]
    if not status & 0x08:
        os.write(descriptor, INITIALIZE_COMMAND)
        time.sleep(0.02)


def read_measurement(descriptor: int) -> dict:
    os.write(descriptor, MEASURE_COMMAND)
    for _ in range(10):
        time.sleep(0.01)
        data = read_exact(descriptor, 7)
        if not data[0] & 0x80:
            break
    else:
        raise TimeoutError("AHT20 conversion timed out")

    if crc8(data[:6]) != data[6]:
        raise ValueError(f"AHT20 CRC mismatch: {data.hex()}")

    humidity_raw = (data[1] << 12) | (data[2] << 4) | (data[3] >> 4)
    temperature_raw = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
    return {
        "temperature_c": round((temperature_raw * 200.0 / 1048576.0) - 50.0, 2),
        "humidity_percent": round(humidity_raw * 100.0 / 1048576.0, 2),
        "timestamp": time.time(),
    }


def write_state(path: str, state: dict) -> None:
    temporary = f"{path}.tmp"
    with open(temporary, "w", encoding="utf-8") as output:
        json.dump(state, output, ensure_ascii=False, separators=(",", ":"))
        output.write("\n")
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser(description="Read AHT20 temperature and humidity over I2C")
    parser.add_argument("--device", default="/dev/i2c-3")
    parser.add_argument("--address", type=lambda value: int(value, 0), default=AHT20_ADDRESS)
    parser.add_argument("--output", default="/run/acoustic-eye/aht20.json")
    parser.add_argument("--interval", type=float, default=2.0)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    descriptor = os.open(args.device, os.O_RDWR)
    try:
        fcntl.ioctl(descriptor, I2C_SLAVE, args.address)
        initialize(descriptor)
        while True:
            try:
                state = read_measurement(descriptor)
                write_state(args.output, state)
                if args.verbose:
                    print(json.dumps(state, ensure_ascii=False), flush=True)
            except (OSError, TimeoutError, ValueError) as error:
                print(f"AHT20 read failed: {error}", file=sys.stderr, flush=True)
            time.sleep(args.interval)
    finally:
        os.close(descriptor)


if __name__ == "__main__":
    raise SystemExit(main())
