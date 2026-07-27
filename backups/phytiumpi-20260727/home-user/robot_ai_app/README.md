# robot_ai_app

ROS 2 native AI control layer for the wheel-leg robot.

This package replaces the older standalone `~/robot_ai_app` HTTP client path. It talks directly to the existing ROS graph:

- publishes motion to `/cmd_vel`
- subscribes robot status from `/wheel_leg/status`
- subscribes camera frames from `/camera/image_raw`
- reads `/odom`
- optionally sends Nav2 goals to the `navigate_to_pose` action

## Safety Model

The AI model never receives permission to continuously control raw motor speeds. It can only choose one whitelisted task/action per turn. Local code then:

- clamps `vx`, `wz`, and duration
- uses Nav2 relative goals for distance, angle, and short `move_base` tasks when navigation is available
- falls back to small `/cmd_vel` steps only when Nav2 is unavailable
- stops after each segment
- falls back to `stop` on execution failure
- blocks object navigation until target detection or semantic map backends are implemented

Default limits:

```text
max_vx       0.10 m/s
max_wz       0.75 rad/s
max_duration 1.00 s
```

## Commands

Keyboard teleop:

```bash
ros2 run robot_ai_app robot_ai_teleop
```

Save one camera frame:

```bash
ros2 run robot_ai_app robot_ai_camera_dump --output frame.jpg
```

Scripted or reactive loop:

```bash
ros2 run robot_ai_app robot_ai_agent --policy scripted --steps 8
ros2 run robot_ai_app robot_ai_agent --policy reactive --steps 20
```

Qwen-VL visual loop:

```bash
export DASHSCOPE_API_KEY="your_key"
export QWEN_MODEL="qwen-vl-max-latest"
ros2 run robot_ai_app robot_ai_agent --policy qwen --steps 10
```

Chinese chat control:

```bash
export DASHSCOPE_API_KEY="your_key"
ros2 run robot_ai_app robot_ai_chat
```

Local voice chat demo, without robot motion output:

```bash
export DASHSCOPE_API_KEY="your_key"
ros2 run robot_ai_app robot_ai_voice_demo --text "先介绍一下你能做什么" --no-tts
pip install sounddevice numpy faster-whisper pyttsx3
ros2 run robot_ai_app robot_ai_voice_demo --seconds 5 --whisper-model base
```

The voice demo records from the microphone, transcribes locally, sends text to
Qwen, prints a safe task preview, and speaks the reply. It does not publish
`/cmd_vel`.

Common topic overrides:

```bash
ros2 run robot_ai_app robot_ai_chat \
  --cmd-vel-topic /cmd_vel \
  --status-topic /wheel_leg/status \
  --camera-topic /camera/image_raw \
  --odom-topic /odom \
  --navigate-action navigate_to_pose
```

## Navigation

`navigate_to_pose` is implemented through Nav2:

```json
{
  "type": "navigate_to_pose",
  "params": {"x": 1.0, "y": 0.5}
}
```

It requires:

- `/map` or live SLAM
- `/odom`
- TF `map -> odom -> base_link`
- Nav2 action server `navigate_to_pose`

Precision motion policy:

- `move_distance`: converted to a relative Nav2 goal in the `map` frame.
- `turn_angle`: executed as an in-place TF-feedback rotation using `/cmd_vel` angular velocity only.
- `move_base`: linear motion is converted to a short relative Nav2 goal; pure angular motion is executed in place.
- `navigate_to_pose`: position-first; final orientation is ignored by Nav2 and handled by a separate in-place rotation only when `yaw` is explicitly provided.
- after Nav2 reports success, the AI executor performs a TF-based final settle to the target point, default tolerance `0.03m`.
- fallback `/cmd_vel` execution is reserved for bringup/debug when Nav2 is not available.

`navigate_to_object` is intentionally a reserved task. It currently captures an observation and returns a clear unsupported result unless a semantic map or detector is added.

## Future Backends

Target detection extension point:

```python
from robot_ai_app.target_detection import TargetDetector
```

Semantic map extension point:

```python
from robot_ai_app.semantic_map import SemanticMap
```

These are designed so object-aware navigation can be added later without changing the chat schema.
