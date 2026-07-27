import struct

from chassis_control_ros2.protocol import (
    decode_chassis_telemetry,
    decode_frame,
    encode_frame,
    track_width_payload,
    velocity_payload,
)


def test_frame_round_trip():
    frame = encode_frame(62, 7, velocity_payload(0.1, -0.2, 300))
    command, sequence, payload = decode_frame(frame)
    assert command == 62
    assert sequence == 7
    assert struct.unpack(">iiH", payload) == (100000, -200000, 300)


def test_chassis_telemetry_decode():
    payload = struct.pack(
        ">BBBBIiiiiiiiii",
        1, 0, 2, 0, 20,
        100000, 200000, 90000, 180000,
        80000, 160000, 300000, 400000, 250000,
    )
    telemetry = decode_chassis_telemetry(payload)
    assert telemetry.balance_state == 2
    assert telemetry.target_linear_m_s == 0.1
    assert telemetry.measured_angular_rad_s == 0.16
    assert telemetry.wheel_track_m == 0.25


def test_track_width_payload():
    assert struct.unpack(">i", track_width_payload(0.25))[0] == 250000
