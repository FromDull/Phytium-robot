#!/usr/bin/env python3
import argparse
import json
import logging
import signal
from dataclasses import asdict

from wifi_screen.config import load_config
from wifi_screen.controller import WifiScreenController
from wifi_screen.network import WifiManager
from wifi_screen.serial_port import SerialPort


def parse_args():
    parser = argparse.ArgumentParser(
        description="Control Phytium Pi Wi-Fi from a TJC serial touch screen"
    )
    parser.add_argument("--config", default="config.json", help="path to JSON config")
    parser.add_argument(
        "--scan",
        action="store_true",
        help="scan Wi-Fi once and print JSON without opening the serial port",
    )
    parser.add_argument(
        "--check-serial",
        action="store_true",
        help="open and configure the serial port, then exit",
    )
    parser.add_argument("--verbose", action="store_true", help="enable debug logging")
    return parser.parse_args()


def main():
    args = parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    config = load_config(args.config)

    if args.scan:
        manager = WifiManager(config.wifi_interface)
        print(json.dumps([asdict(item) for item in manager.scan()], ensure_ascii=False, indent=2))
        return

    if args.check_serial:
        port = SerialPort(config.serial_device, config.baudrate)
        try:
            port.open()
            print(f"serial OK: {config.serial_device} at {config.baudrate} baud")
        finally:
            port.close()
        return

    controller = WifiScreenController(config)
    signal.signal(signal.SIGINT, lambda *_: controller.stop())
    signal.signal(signal.SIGTERM, lambda *_: controller.stop())
    controller.run()


if __name__ == "__main__":
    main()
