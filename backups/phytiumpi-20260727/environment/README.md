# Runtime environment backup

This directory extends the source snapshot with the board's recoverable runtime environment as captured on 2026-07-27.

## Contents

- `home-user-dotfiles/`: `.bashrc`, `.profile`, aliases, editor/input settings, and non-secret Git configuration.
- `system-config/`: readable custom system configuration, modified Debian conffiles, system integration files, and sanitized NetworkManager state.
- `metadata/`: installed Debian/Python packages, Docker image inventory, enabled units, storage/network/kernel state, modified-conffile evidence, model inventory, and archive checksums.
- `models/`: complete `/home/user/models` snapshot, including Paraformer, Faster Whisper, KWS, and YOLO weights.
- `archives/voice-env.tar.gz`: complete `/home/user/voice-env` archive.
- `archives/tts-runtime.tar.gz`: complete `/home/user/tts-runtime` archive.

Model files and environment archives are stored with Git LFS because several objects exceed GitHub's normal 100 MB file limit. Run `git lfs pull` after cloning.

## Deliberate security exclusions

The backup does not contain SSH private keys, `authorized_keys`, API keys, live `.env` values, browser-terminal password hashes, `/etc/shadow`, Docker's private key, TLS private keys, shell history, or NetworkManager passwords. `voice.env.example` retains variable names with redacted values. NetworkManager profiles were root-only, so non-secret connection state was exported through `nmcli`; Wi-Fi credentials must be restored separately.

## Restore outline

```bash
git lfs pull
sudo tar -xpf archives/voice-env.tar.gz -C /home/user
sudo tar -xpf archives/tts-runtime.tar.gz -C /home/user
```

Restore configuration selectively after reviewing differences. Do not copy `system-config/` wholesale over a running operating system. Recreate secrets through a secure channel, verify absolute paths and ownership, then run `systemctl daemon-reload` and restart only the affected services.

The virtual environments were archived for exact recovery, but rebuilding from `metadata/*pip-freeze.txt` is preferable after a Python/OS upgrade because compiled wheels are architecture- and interpreter-specific.
