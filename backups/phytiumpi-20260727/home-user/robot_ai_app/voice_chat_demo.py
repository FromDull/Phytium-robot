"""Voice chat demo that does not publish robot commands.

Pipeline:
microphone -> local speech-to-text -> Qwen API -> local text-to-speech.

This is intentionally separate from ROS control so it can be tested safely on a
desktop before the robot base and camera are available.
"""

from __future__ import annotations

import argparse
import asyncio
from collections import deque
from functools import lru_cache
import math
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile
import time
import wave

from .chat_policy import QwenChatPolicy
from .intent_parser import parse_user_intent
from .qwen_policy import QwenPolicyError


DEMO_STATE = {
    "status": {},
    "command": {},
    "camera": {"available": False},
    "nav": {"available": False},
    "limits": {"max_vx": 0.10, "max_wz": 0.75, "max_duration": 1.0},
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Local STT + Qwen + local TTS demo.")
    parser.add_argument("--seconds", type=float, default=5.0, help="Recording duration for each turn.")
    parser.add_argument("--samplerate", type=int, default=16000)
    parser.add_argument(
        "--input-device",
        help="sounddevice input device index or name (for example, ASTRA Pro).",
    )
    parser.add_argument(
        "--alsa-device",
        help="Record with arecord from this ALSA device (for example, hw:CARD=Pro,DEV=0).",
    )
    parser.add_argument("--vad", action="store_true", help="Listen continuously and stop after speech ends.")
    parser.add_argument("--vad-threshold", type=float, default=0.018, help="Normalized RMS speech threshold.")
    parser.add_argument("--vad-silence", type=float, default=0.8, help="Silence seconds that end an utterance.")
    parser.add_argument("--vad-max-seconds", type=float, default=12.0, help="Maximum utterance length.")
    parser.add_argument("--vad-cooldown", type=float, default=0.7, help="Delay before listening after playback.")
    parser.add_argument("--vad-channel", type=int, choices=[0, 1], default=0, help="Astra microphone channel.")
    parser.add_argument("--language", default="zh", help="STT language hint.")
    parser.add_argument("--stt-backend", choices=["auto", "faster-whisper", "whisper"], default="auto")
    parser.add_argument("--whisper-model", default="base", help="Whisper/faster-whisper model name.")
    parser.add_argument("--text", help="Send this text directly instead of recording/transcribing audio.")
    parser.add_argument("--audio-file", help="Transcribe this file instead of recording from microphone.")
    parser.add_argument("--save-last-audio", help="Copy the latest recorded utterance to this WAV path.")
    parser.add_argument("--llm-timeout", type=float, default=30.0)
    parser.add_argument("--no-tts", action="store_true")
    parser.add_argument("--once", action="store_true", help="Run one voice turn and exit.")
    return parser


def record_wav(path: Path, seconds: float, samplerate: int, input_device: str | None = None) -> None:
    try:
        import numpy as np  # type: ignore
        import sounddevice as sd  # type: ignore
    except ImportError as exc:
        raise RuntimeError("recording needs: pip install sounddevice numpy") from exc

    print(f"Recording {seconds:.1f}s... speak now.")
    device: int | str | None = input_device
    if input_device is not None and input_device.isdigit():
        device = int(input_device)
    audio = sd.rec(
        int(seconds * samplerate),
        samplerate=samplerate,
        channels=1,
        dtype="float32",
        device=device,
    )
    sd.wait()
    pcm = np.clip(audio[:, 0], -1.0, 1.0)
    pcm16 = (pcm * 32767.0).astype(np.int16)
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(samplerate)
        wav.writeframes(pcm16.tobytes())


def record_wav_with_arecord(path: Path, seconds: float, samplerate: int, device: str) -> None:
    duration = max(1, math.ceil(seconds))
    print(f"Recording {duration}s... speak now.")
    subprocess.run(
        [
            "arecord",
            "-q",
            "-D",
            device,
            "-d",
            str(duration),
            "-f",
            "S16_LE",
            "-r",
            str(samplerate),
            "-c",
            "2",
            str(path),
        ],
        check=True,
        timeout=duration + 10,
    )


def record_wav_with_vad(
    path: Path,
    samplerate: int,
    device: str,
    threshold: float,
    silence_seconds: float,
    max_seconds: float,
    channel: int,
) -> None:
    try:
        import numpy as np  # type: ignore
    except ImportError as exc:
        raise RuntimeError("VAD recording needs: pip install numpy") from exc

    channels = 2
    block_frames = max(160, int(samplerate * 0.03))
    block_bytes = block_frames * channels * 2
    pre_roll = deque(maxlen=max(1, int(0.3 / 0.03)))
    captured: list[object] = []
    started = False
    candidate_blocks = 0
    voiced_blocks = 0
    silent_blocks = 0
    silence_blocks = max(1, int(silence_seconds / 0.03))
    max_blocks = max(1, int(max_seconds / 0.03))
    trigger_blocks = 4
    minimum_voiced_blocks = max(trigger_blocks, int(0.3 / 0.03))

    print("Calibrating ambient noise... stay quiet for 1 second.")
    process = subprocess.Popen(
        [
            "arecord",
            "-q",
            "-D",
            device,
            "-f",
            "S16_LE",
            "-r",
            str(samplerate),
            "-c",
            str(channels),
            "-t",
            "raw",
        ],
        stdout=subprocess.PIPE,
    )
    if process.stdout is None:
        raise RuntimeError("arecord stdout is unavailable")
    try:
        noise_levels: list[float] = []
        for _ in range(max(1, int(1.0 / 0.03))):
            raw = process.stdout.read(block_bytes)
            if len(raw) != block_bytes:
                raise RuntimeError("arecord stopped during VAD calibration")
            stereo = np.frombuffer(raw, dtype=np.int16).reshape(-1, channels)
            mono = stereo[:, channel].astype(np.float32)
            noise_levels.append(float(np.sqrt(np.mean(mono * mono)) / 32768.0))
        noise_floor = float(np.percentile(noise_levels, 90))
        effective_threshold = max(threshold, noise_floor * 2.5)
        print(
            f"Listening... noise={noise_floor:.4f}, threshold={effective_threshold:.4f}. "
            "Speak when ready."
        )

        while True:
            raw = process.stdout.read(block_bytes)
            if len(raw) != block_bytes:
                raise RuntimeError("arecord stopped before VAD completed")
            stereo = np.frombuffer(raw, dtype=np.int16).reshape(-1, channels)
            mono = stereo[:, channel].astype(np.float32)
            rms = float(np.sqrt(np.mean(mono * mono)) / 32768.0)
            mono16 = np.clip(mono, -32768, 32767).astype(np.int16)

            if not started:
                pre_roll.append(mono16)
                candidate_blocks = candidate_blocks + 1 if rms >= effective_threshold else 0
                if candidate_blocks >= trigger_blocks:
                    print("Speech detected.")
                    started = True
                    captured.extend(pre_roll)
                    voiced_blocks = candidate_blocks
                continue

            captured.append(mono16)
            if rms < effective_threshold:
                silent_blocks += 1
            else:
                silent_blocks = 0
                voiced_blocks += 1
            if silent_blocks >= silence_blocks:
                if voiced_blocks < minimum_voiced_blocks:
                    print("Ignored a short noise trigger.")
                    captured.clear()
                    pre_roll.clear()
                    started = False
                    candidate_blocks = 0
                    voiced_blocks = 0
                    silent_blocks = 0
                    continue
                break
            if len(captured) >= max_blocks:
                break
    finally:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()

    pcm16 = np.concatenate(captured) if captured else np.zeros(block_frames, dtype=np.int16)
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(samplerate)
        wav.writeframes(pcm16.tobytes())


def transcribe_audio(path: Path, backend: str, model_name: str, language: str) -> str:
    errors: list[str] = []
    backends = ["faster-whisper", "whisper"] if backend == "auto" else [backend]
    for item in backends:
        try:
            if item == "faster-whisper":
                return transcribe_with_faster_whisper(path, model_name, language)
            if item == "whisper":
                return transcribe_with_whisper(path, model_name, language)
        except ImportError as exc:
            errors.append(f"{item}: missing dependency {exc}")
        except Exception as exc:
            errors.append(f"{item}: {exc}")
    raise RuntimeError("STT failed. " + " | ".join(errors))


def transcribe_with_faster_whisper(path: Path, model_name: str, language: str) -> str:
    model = load_faster_whisper_model(model_name)
    segments, _info = model.transcribe(
        str(path),
        language=language,
        beam_size=1,
        best_of=1,
        condition_on_previous_text=False,
    )
    return "".join(segment.text for segment in segments).strip()


@lru_cache(maxsize=2)
def load_faster_whisper_model(model_name: str):
    from faster_whisper import WhisperModel  # type: ignore

    return WhisperModel(model_name, device="cpu", compute_type="int8", cpu_threads=4, num_workers=1)


def transcribe_with_whisper(path: Path, model_name: str, language: str) -> str:
    model = load_whisper_model(model_name)
    result = model.transcribe(str(path), language=language)
    return str(result.get("text", "")).strip()


@lru_cache(maxsize=2)
def load_whisper_model(model_name: str):
    import whisper  # type: ignore

    return whisper.load_model(model_name)


def speak(text: str, enabled: bool = True) -> None:
    if not enabled or not text:
        return
    if platform.system() == "Linux":
        if speak_with_edge_tts(text):
            return
        print("TTS unavailable: edge-tts or mpv failed.")
        return
    if speak_with_pyttsx3(text):
        return
    if platform.system() == "Windows":
        ps = (
            "Add-Type -AssemblyName System.Speech; "
            "$s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
            f"$s.Speak({text!r})"
        )
        subprocess.run(["powershell", "-NoProfile", "-Command", ps], check=False)
        return
    for cmd in (["say", text],):
        try:
            subprocess.run(cmd, check=False)
            return
        except FileNotFoundError:
            continue
    print("TTS unavailable. Install pyttsx3 or espeak-ng.")


def speak_with_edge_tts(text: str) -> bool:
    voice = os.getenv("EDGE_TTS_VOICE", "zh-CN-XiaoxiaoNeural")
    rate = os.getenv("EDGE_TTS_RATE", "+25%")
    audio_device = os.getenv("TTS_AUDIO_DEVICE", "alsa/plughw:0,0")
    try:
        asyncio.run(stream_edge_tts(text, voice, rate, audio_device))
        return True
    except (ImportError, FileNotFoundError, OSError, subprocess.SubprocessError):
        return False


async def stream_edge_tts(text: str, voice: str, rate: str, audio_device: str) -> None:
    import edge_tts  # type: ignore

    player = subprocess.Popen(
        ["mpv", "--no-video", "--really-quiet", "--cache=no", f"--audio-device={audio_device}", "-"],
        stdin=subprocess.PIPE,
    )
    if player.stdin is None:
        raise RuntimeError("mpv stdin is unavailable")
    try:
        communicate = edge_tts.Communicate(text, voice, rate=rate)
        started = False
        async for chunk in communicate.stream():
            if chunk["type"] != "audio":
                continue
            if not started:
                print("TTS stream started.")
                started = True
            player.stdin.write(chunk["data"])
            player.stdin.flush()
    finally:
        player.stdin.close()
        player.wait(timeout=30)
    if player.returncode != 0:
        raise subprocess.CalledProcessError(player.returncode, player.args)


def speak_with_pyttsx3(text: str) -> bool:
    try:
        import pyttsx3  # type: ignore
    except ImportError:
        return False
    engine = pyttsx3.init()
    engine.say(text)
    engine.runAndWait()
    return True


def run_turn(args: argparse.Namespace, policy: QwenChatPolicy, history: list[dict[str, str]]) -> bool:
    turn_started = time.perf_counter()
    if args.text:
        user_text = args.text
        stt_elapsed = 0.0
    else:
        with tempfile.TemporaryDirectory() as tmp:
            audio_path = Path(args.audio_file) if args.audio_file else Path(tmp) / "turn.wav"
            if args.audio_file is None:
                if args.vad:
                    if not args.alsa_device:
                        raise RuntimeError("--vad requires --alsa-device")
                    record_wav_with_vad(
                        audio_path,
                        args.samplerate,
                        args.alsa_device,
                        args.vad_threshold,
                        args.vad_silence,
                        args.vad_max_seconds,
                        args.vad_channel,
                    )
                elif args.alsa_device:
                    record_wav_with_arecord(audio_path, args.seconds, args.samplerate, args.alsa_device)
                else:
                    record_wav(audio_path, args.seconds, args.samplerate, args.input_device)
            if args.save_last_audio and args.audio_file is None:
                shutil.copyfile(audio_path, Path(args.save_last_audio).expanduser())
            print("Transcribing speech...")
            stt_started = time.perf_counter()
            user_text = transcribe_audio(audio_path, args.stt_backend, args.whisper_model, args.language)
            stt_elapsed = time.perf_counter() - stt_started

    if not user_text:
        print("STT> no speech detected")
        return True
    if user_text.lower() in {"q", "quit", "exit"}:
        return False

    print(f"你> {user_text}")
    intent = parse_user_intent(user_text)
    print("Waiting for AI response...")
    llm_started = time.perf_counter()
    decision = policy.decide_from_user(user_text, DEMO_STATE, image_path=None, history=history, intent=intent)
    llm_elapsed = time.perf_counter() - llm_started
    print(f"AI> {decision.reply}")
    print(f"任务预览> {decision.task.type if decision.task else decision.action} | 安全> {decision.safety} | 原因> {decision.reason}")
    if not args.no_tts:
        print("Generating speech...")
    tts_started = time.perf_counter()
    speak(decision.reply, enabled=not args.no_tts)
    tts_elapsed = time.perf_counter() - tts_started
    print(
        f"Timing> STT {stt_elapsed:.2f}s | API {llm_elapsed:.2f}s | "
        f"TTS {tts_elapsed:.2f}s | total {time.perf_counter() - turn_started:.2f}s"
    )
    history.append({"role": "user", "content": user_text})
    history.append({"role": "assistant", "content": decision.reply})
    return True


def main() -> int:
    args = build_parser().parse_args()
    if not os.getenv("DASHSCOPE_API_KEY"):
        print("DASHSCOPE_API_KEY is not set.")
        return 2
    try:
        policy = QwenChatPolicy(timeout=args.llm_timeout)
    except QwenPolicyError as exc:
        print(exc)
        return 2

    print("Voice chat demo. It will not publish /cmd_vel or control the robot.")
    history: list[dict[str, str]] = []
    try:
        while True:
            if args.audio_file is None and args.text is None and not args.vad:
                answer = input("Press Enter to record, or q to quit: ").strip().lower()
                if answer in {"q", "quit", "exit"}:
                    return 0
            if not run_turn(args, policy, history):
                return 0
            if args.vad:
                time.sleep(args.vad_cooldown)
            if args.once or args.audio_file or args.text:
                return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"voice demo failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
