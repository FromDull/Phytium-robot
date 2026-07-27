# Phytium Pi source backup

- Captured: 2026-07-27 (Asia/Shanghai)
- Source host: `user@192.168.137.129` (`phytiumpi`)
- Purpose: Git-friendly backup of the currently deployed robot source code, deployment scripts, service definitions, and non-secret configuration.

## Layout

- `home-user/`: selected source trees originally under `/home/user`.
- `etc/systemd/system/`: custom system service units and drop-ins.
- `home-user/.config/systemd/user/`: user service units and drop-ins.
- `usr-local/sbin/`, `usr-local/libexec/`, `usr-local/share/rpmsg-monitor/`: deployed helper scripts and monitor assets.
- `SOURCE_MANIFEST.tsv`: relative path, byte size, and SHA-256 for every backed-up file.

## Included application trees

- AcousticEye I2C readers
- Astra/ROS 2 camera and target-localization source
- CH341 kernel module source
- gimbal daemon
- OpenAMP/RPMsg firmware, Linux clients, broker, tests, ROS 2 integration, and LQR scripts
- robot AI application: VAD, KWS/STT, Qwen policy, TTS, safety, perception, expression, and navigation bridge
- RPLIDAR, odometry, SLAM, Nav2 preview, dashboard, and deployment scripts
- Wi-Fi/TJC screen controller
- custom systemd and `/usr/local` deployment helpers

## Intentionally excluded

This is a source backup, not a disk image. The collection process excluded:

- `.git` metadata from nested/upstream repositories;
- API keys, `.env`/`voice.env`, SSH material, password databases, private keys, and certificates;
- model weights (`.onnx`, `.bin`, `.pt`, etc.);
- Python virtual environments and package caches;
- ROS/build/install/log directories, object files, kernel modules, executables, and ELF firmware;
- runtime data, audio/video/images, CSV logs, map posegraphs, archives, Docker images, and historical backup directories.

Important external runtime assets that must be restored separately include the Paraformer and KWS models, YOLO weights, OpenNI vendor libraries, `openamp_core0.elf`, Python environment packages, Docker images, dashboard terminal password hash, and `DASHSCOPE_API_KEY`.

## Restore guidance

Do not copy this directory wholesale onto a running board. Restore each subtree to its original absolute path, inspect differences, rebuild native components, restore secrets from a secure channel, then reload/restart only the affected services. Nested third-party source trees are snapshots without their original Git history; upstream origins are documented in their own project files where available.

## Security scan

Before publishing, the snapshot was scanned for common private-key headers and token prefixes (`sk-`, `gh*`, AWS access keys, long Bearer tokens). No live credential was detected. Code-level fields and placeholders related to passwords/tokens remain because they are part of the implementation.
