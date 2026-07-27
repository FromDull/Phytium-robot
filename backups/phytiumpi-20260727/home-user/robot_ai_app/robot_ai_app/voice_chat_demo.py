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
import re
import shutil
import subprocess
import sys
import tempfile
import time
import wave
from typing import Callable

from .chat_policy import QwenChatPolicy
from .acoustic_direction import AcousticDirectionTracker
from .intent_parser import parse_user_intent
from .gimbal_voice import execute_gimbal_voice_command, execute_look_at_me, refine_person_alignment
from .qwen_policy import QwenPolicyError
from .target_detection import TargetDetector
from .expression_bridge import ExpressionBridge


DEMO_STATE = {
    "status": {},
    "command": {},
    "camera": {"available": False},
    "nav": {"available": False},
    "limits": {"max_vx": 0.10, "max_wz": 0.75, "max_duration": 1.0},
}

_VAD_THRESHOLD_CACHE: dict[tuple[str, int, int], tuple[float, float]] = {}


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
    parser.add_argument(
        "--vad-max-threshold",
        type=float,
        default=0.040,
        help="Maximum adaptive VAD threshold, preventing noisy calibration from disabling detection.",
    )
    parser.add_argument("--vad-silence", type=float, default=0.8, help="Silence seconds that end an utterance.")
    parser.add_argument("--vad-max-seconds", type=float, default=5.0, help="Maximum utterance length.")
    parser.add_argument("--vad-cooldown", type=float, default=2.5, help="Delay before listening after playback.")
    parser.add_argument("--vad-channel", type=int, choices=[0, 1], default=0, help="Astra microphone channel.")
    parser.add_argument("--paraformer-model", default="", help="Local Paraformer ONNX model directory.")
    parser.add_argument("--kws-model-dir", default="", help="Sherpa-ONNX keyword spotter model directory.")
    parser.add_argument("--kws-keywords-file", default="", help="Tokenized keyword list used before full STT.")
    parser.add_argument("--kws-threshold", type=float, default=0.25)
    parser.add_argument("--kws-score", type=float, default=1.5)
    parser.add_argument("--kws-threads", type=int, default=1)
    parser.add_argument("--text", help="Send this text directly instead of recording/transcribing audio.")
    parser.add_argument("--audio-file", help="Transcribe this file instead of recording from microphone.")
    parser.add_argument("--save-last-audio", help="Copy the latest recorded utterance to this WAV path.")
    parser.add_argument(
        "--camera-bridge-dir",
        help="Shared directory used to request an RGB frame for visual questions.",
    )
    parser.add_argument(
        "--wake-word",
        action="append",
        default=[],
        help="Require this wake word before handling speech. May be specified more than once.",
    )
    parser.add_argument("--llm-timeout", type=float, default=30.0)
    parser.add_argument("--gimbalctl", default="/usr/local/bin/gimbalctl")
    parser.add_argument("--gimbal-step-deg", type=float, default=2.0)
    parser.add_argument("--doa-shadow", action="store_true", help="Log wake-word DoA without moving the gimbal.")
    parser.add_argument("--doa-state-file", default="/run/acoustic-eye/angle.json")
    parser.add_argument("--doa-window-seconds", type=float, default=2.0)
    parser.add_argument(
        "--doa-wake-window-seconds",
        type=float,
        default=0.6,
        help="Short DoA window aligned to the detected wake phrase.",
    )
    parser.add_argument("--perception-state-url", default="http://127.0.0.1:8080/api/state")
    parser.add_argument("--yolo-detections-url", default="http://127.0.0.1:8091/detections")
    parser.add_argument("--no-tts", action="store_true")
    parser.add_argument("--face-map", help="JSON mapping from voice states to screen expressions.")
    parser.add_argument("--face-socket", default="/run/wifi-screen/face.sock")
    parser.add_argument("--once", action="store_true", help="Run one voice turn and exit.")
    return parser


def extract_wake_command(text: str, wake_words: list[str]) -> str | None:
    """Return speech after a four-character wake phrase."""
    if not wake_words:
        return text.strip()

    tolerant_patterns = (
        r"(?:\u4f60\u597d|\u60a8\u597d)\u5c0f[\u98de\u98db\u975e\u83f2\u80ba\u8f89\u7070\u6062\u9ed1vVcC]",
        r"\u4f60\u8981\u597d\u5c0f[\u98de\u98db\u975e\u83f2\u80ba]",
    )
    for pattern in tolerant_patterns:
        match = re.search(pattern, text)
        if match:
            return text[match.end():].lstrip(" \t,\uFF0C\u3002.!\uFF01?\uFF1F:\uFF1A")

    for wake_word in wake_words:
        word = wake_word.strip()
        if len(word) < 4:
            continue
        match = re.search(re.escape(word), text, flags=re.IGNORECASE)
        if match:
            return text[match.end():].lstrip(" \t,\uFF0C\u3002.!\uFF01?\uFF1F:\uFF1A")
    return None


def should_use_camera(text: str, intent) -> bool:
    if getattr(intent, "action", None) == "look":
        return True
    return any(word in text for word in ("看看", "看一下", "观察", "前面有什么", "画面", "摄像头"))


def request_camera_image(shared_dir: str, timeout: float = 6.0) -> str | None:
    directory = Path(shared_dir).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    request_path = directory / "camera_capture.request"
    ready_path = directory / "camera_capture.ready"
    ppm_path = directory / "latest_color.ppm"
    jpg_path = directory / "latest_color.jpg"
    request_token = str(time.time_ns())
    ready_path.unlink(missing_ok=True)
    request_path.write_text(request_token, encoding="ascii")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ready = None
        try:
            if ready_path.exists():
                ready = ready_path.read_text(encoding="ascii").strip()
        except OSError:
            pass
        if ready == request_token and ppm_path.exists():
            try:
                from PIL import Image  # type: ignore

                with Image.open(ppm_path) as image:
                    image.save(jpg_path, "JPEG", quality=88)
                return str(jpg_path)
            except ImportError:
                print("Vision needs Pillow: pip install pillow")
                return None
        time.sleep(0.05)
    print("Vision> camera capture timed out")
    return None


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
    max_threshold: float,
    silence_seconds: float,
    max_seconds: float,
    channel: int,
    on_speech_detected: Callable[[], None] | None = None,
) -> tuple[float, float]:
    try:
        import numpy as np  # type: ignore
    except ImportError as exc:
        raise RuntimeError("VAD recording needs: pip install numpy") from exc

    channels = 2
    block_frames = max(160, int(samplerate * 0.03))
    block_bytes = block_frames * channels * 2
    pre_roll = deque(maxlen=max(1, int(1.0 / 0.03)))
    captured: list[object] = []
    started = False
    speech_started_at = 0.0
    candidate_blocks = 0
    voiced_blocks = 0
    silent_blocks = 0
    silence_blocks = max(1, int(silence_seconds / 0.03))
    max_blocks = max(1, int(max_seconds / 0.03))
    trigger_blocks = 4
    minimum_voiced_blocks = max(trigger_blocks, int(0.3 / 0.03))

    cache_key = (device, samplerate, channel)
    cached_threshold = _VAD_THRESHOLD_CACHE.get(cache_key)
    if cached_threshold is None:
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
        if cached_threshold is None:
            noise_levels: list[float] = []
            for _ in range(max(1, int(1.0 / 0.03))):
                raw = process.stdout.read(block_bytes)
                if len(raw) != block_bytes:
                    raise RuntimeError("arecord stopped during VAD calibration")
                stereo = np.frombuffer(raw, dtype=np.int16).reshape(-1, channels)
                mono = stereo[:, channel].astype(np.float32)
                mono -= np.mean(mono)
                noise_levels.append(float(np.sqrt(np.mean(mono * mono)) / 32768.0))
            measured_noise_floor = float(np.percentile(noise_levels, 90))
            if measured_noise_floor * 1.5 >= max_threshold:
                # Someone spoke or a transient played during calibration. A
                # clamped maximum would make normal speech impossible to
                # trigger, so reject the contaminated sample and use the
                # configured baseline until the next process start.
                print(
                    f"VAD calibration contaminated (noise={measured_noise_floor:.4f}); "
                    f"using baseline threshold={threshold:.4f}."
                )
                noise_floor = threshold / 1.5
                effective_threshold = threshold
            else:
                noise_floor = measured_noise_floor
                effective_threshold = max(threshold, noise_floor * 1.5)
            _VAD_THRESHOLD_CACHE[cache_key] = (noise_floor, effective_threshold)
        else:
            noise_floor, effective_threshold = cached_threshold
        # Ambient fan/room noise can drift upward after the one-second
        # calibration. Keep release close enough to the trigger threshold to
        # end an utterance, while retaining hysteresis for quiet syllables.
        release_threshold = min(
            effective_threshold * 0.97,
            max(noise_floor * 1.4, effective_threshold * 0.94),
        )
        print(
            f"Listening... noise={noise_floor:.4f}, threshold={effective_threshold:.4f}, "
            f"release={release_threshold:.4f}. "
            "Speak when ready."
        )

        while True:
            raw = process.stdout.read(block_bytes)
            if len(raw) != block_bytes:
                raise RuntimeError("arecord stopped before VAD completed")
            stereo = np.frombuffer(raw, dtype=np.int16).reshape(-1, channels)
            mono = stereo[:, channel].astype(np.float32)
            mono -= np.mean(mono)
            rms = float(np.sqrt(np.mean(mono * mono)) / 32768.0)
            mono16 = np.clip(mono, -32768, 32767).astype(np.int16)

            if not started:
                pre_roll.append(mono16)
                candidate_blocks = candidate_blocks + 1 if rms >= effective_threshold else 0
                if candidate_blocks >= trigger_blocks:
                    print("Speech detected.")
                    if on_speech_detected is not None:
                        on_speech_detected()
                    started = True
                    speech_started_at = time.time() - candidate_blocks * 0.03
                    captured.extend(pre_roll)
                    voiced_blocks = candidate_blocks
                continue

            captured.append(mono16)
            # Once speech starts, use a lower release threshold so quiet syllables
            # and sentence endings do not split one utterance into short triggers.
            if rms < release_threshold:
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
    print(f"Captured speech audio: {len(pcm16) / samplerate:.2f}s")
    return speech_started_at, time.time()


def record_wav_with_vad_retry(
    path: Path,
    samplerate: int,
    device: str,
    threshold: float,
    max_threshold: float,
    silence_seconds: float,
    max_seconds: float,
    channel: int,
    attempts: int = 3,
    on_speech_detected: Callable[[], None] | None = None,
) -> tuple[float, float]:
    for attempt in range(1, attempts + 1):
        try:
            return record_wav_with_vad(
                path,
                samplerate,
                device,
                threshold,
                max_threshold,
                silence_seconds,
                max_seconds,
                channel,
                on_speech_detected,
            )
            return
        except RuntimeError as error:
            if "arecord stopped" not in str(error) or attempt >= attempts:
                raise
            print(f"Microphone read failed; retrying ({attempt}/{attempts})...")
            time.sleep(1.5)
    raise RuntimeError("VAD recording attempts exhausted")


def prepare_stt_audio(path: Path) -> Path:
    """Create a cleaned speech file for recognition."""
    import subprocess

    cleaned_path = path.with_name(f"{path.stem}_clean.wav")
    command = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-i", str(path),
        "-af", "highpass=f=120,lowpass=f=7500,afftdn=nf=-30,dynaudnorm=f=150:g=8:p=0.9",
        "-ar", "16000", "-ac", "1",
        str(cleaned_path),
    ]
    try:
        subprocess.run(command, check=True, capture_output=True, text=True)
        return cleaned_path
    except (OSError, subprocess.CalledProcessError) as error:
        print(f"Audio cleanup failed; using raw recording: {error}")
        return path


@lru_cache(maxsize=2)
def load_keyword_spotter(
    model_dir: str,
    keywords_file: str,
    num_threads: int,
    keywords_score: float,
    keywords_threshold: float,
):
    import sherpa_onnx  # type: ignore

    directory = Path(model_dir)
    return sherpa_onnx.KeywordSpotter(
        tokens=str(directory / "tokens.txt"),
        encoder=str(directory / "encoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx"),
        decoder=str(directory / "decoder-epoch-12-avg-2-chunk-16-left-64.onnx"),
        joiner=str(directory / "joiner-epoch-12-avg-2-chunk-16-left-64.int8.onnx"),
        keywords_file=keywords_file,
        num_threads=max(1, num_threads),
        keywords_score=keywords_score,
        keywords_threshold=keywords_threshold,
        provider="cpu",
    )


def detect_wake_word_audio_with_timing(
    path: Path,
    model_dir: str,
    keywords_file: str,
    num_threads: int = 1,
    keywords_score: float = 1.5,
    keywords_threshold: float = 0.25,
) -> tuple[str, float | None, float | None]:
    """Return the keyword and its first/last token times in the WAV file."""
    import numpy as np  # type: ignore

    spotter = load_keyword_spotter(
        model_dir, keywords_file, max(1, num_threads), keywords_score, keywords_threshold
    )
    with wave.open(str(path), "rb") as wav:
        if wav.getsampwidth() != 2:
            raise ValueError("KWS requires 16-bit PCM WAV audio")
        sample_rate = wav.getframerate()
        channels = wav.getnchannels()
        pcm = np.frombuffer(wav.readframes(wav.getnframes()), dtype=np.int16)
    if channels > 1:
        pcm = pcm.reshape(-1, channels)[:, 0]
    samples = pcm.astype(np.float32) / 32768.0
    stream = spotter.create_stream()
    stream.accept_waveform(sample_rate, samples)
    stream.accept_waveform(sample_rate, np.zeros(int(0.66 * sample_rate), dtype=np.float32))
    stream.input_finished()
    while spotter.is_ready(stream):
        spotter.decode_stream(stream)
        result = spotter.keyword_spotter.get_result(stream)
        keyword = result.keyword.strip()
        if keyword:
            timestamps = list(result.timestamps)
            spotter.reset_stream(stream)
            if timestamps:
                return keyword, float(timestamps[0]), float(timestamps[-1])
            return keyword, None, None
    return "", None, None


def detect_wake_word_audio(
    path: Path,
    model_dir: str,
    keywords_file: str,
    num_threads: int = 1,
    keywords_score: float = 1.5,
    keywords_threshold: float = 0.25,
) -> str:
    """Compatibility wrapper returning only the detected keyword."""
    keyword, _start, _end = detect_wake_word_audio_with_timing(
        path, model_dir, keywords_file, num_threads, keywords_score, keywords_threshold
    )
    return keyword


def transcribe_audio(path: Path, paraformer_model: str) -> str:
    stt_path = prepare_stt_audio(path)
    return transcribe_with_paraformer(stt_path, paraformer_model)


def transcribe_with_paraformer(path: Path, model_dir: str) -> str:
    if not model_dir:
        raise ValueError("Paraformer model directory is not configured")
    model = load_paraformer_model(model_dir)
    result = model([str(path)])
    if not result:
        return ""
    prediction = result[0].get("preds", "")
    if isinstance(prediction, tuple):
        prediction = prediction[0]
    return str(prediction).replace(" ", "").strip()


@lru_cache(maxsize=1)
def load_paraformer_model(model_dir: str):
    from funasr_onnx import Paraformer  # type: ignore

    return Paraformer(
        model_dir,
        batch_size=1,
        quantize=True,
        intra_op_num_threads=2,
    )


def limit_tts_text(text: str, max_chars_per_sentence: int = 25, max_sentences: int = 2) -> str:
    """Bound spoken output without altering the full answer shown or stored."""
    normalized = re.sub(r"\s+", " ", text).strip()
    if not normalized or max_chars_per_sentence <= 0 or max_sentences <= 0:
        return ""

    spoken: list[str] = []
    for unit in re.findall(r"[^。！？!?；;\n]+[。！？!?；;]?", normalized):
        unit = unit.strip()
        if not unit:
            continue
        ending = unit[-1] if unit[-1] in "。！？!?；;" else ""
        content = unit[:-1].strip() if ending else unit
        while content and len(spoken) < max_sentences:
            chunk = content[:max_chars_per_sentence]
            content = content[max_chars_per_sentence:]
            spoken.append(chunk + (ending if not content and ending else "。"))
        if len(spoken) >= max_sentences:
            break
    return "".join(spoken)


def speak(text: str, enabled: bool = True) -> None:
    if not enabled or not text:
        return
    text = limit_tts_text(text)
    if not text:
        return
    if platform.system() == "Linux":
        if speak_with_edge_tts(text):
            return
        print("Network TTS failed; falling back to local TTS.", file=sys.stderr)
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
    if platform.system() == "Linux":
        local_tts_bin = os.getenv("ESPEAK_NG_BIN", "espeak-ng")
        local_tts_voice = os.getenv("ESPEAK_NG_VOICE", "zh")
        local_commands = ([local_tts_bin, "-v", local_tts_voice, text],)
        local_tts_env = None
        bundled_root = Path("/home/user/tts-runtime/root")
        bundled_bin = bundled_root / "usr/bin/espeak-ng"
        if local_tts_bin == "espeak-ng" and bundled_bin.exists():
            local_commands = ([str(bundled_bin), "-v", local_tts_voice, text],)
            local_tts_env = os.environ.copy()
            local_tts_env["ESPEAK_DATA_PATH"] = str(bundled_root / "usr/lib/aarch64-linux-gnu/espeak-ng-data")
            lib_dir = str(bundled_root / "usr/lib/aarch64-linux-gnu")
            local_tts_env["LD_LIBRARY_PATH"] = ":".join(
                part for part in (lib_dir, local_tts_env.get("LD_LIBRARY_PATH", "")) if part
            )
    else:
        local_commands = (["say", text],)
        local_tts_env = None
    for cmd in local_commands:
        try:
            result = subprocess.run(cmd, check=False, env=local_tts_env)
            if result.returncode == 0:
                return
        except (FileNotFoundError, OSError, subprocess.SubprocessError) as error:
            print(f"Local TTS command failed: {error}", file=sys.stderr)
            continue
    print("TTS unavailable; skipping playback and continuing.", file=sys.stderr)


def speak_with_edge_tts(text: str) -> bool:
    voice = os.getenv("EDGE_TTS_VOICE", "zh-CN-XiaoxiaoNeural")
    rate = os.getenv("EDGE_TTS_RATE", "+25%")
    audio_device = os.getenv("TTS_AUDIO_DEVICE", "alsa/plughw:0,0")
    try:
        asyncio.run(stream_edge_tts(text, voice, rate, audio_device))
        return True
    except Exception as error:
        # Provider-specific failures must not terminate the long-running service.
        print(f"Network TTS failed: {type(error).__name__}: {error}", file=sys.stderr)
        return False


async def stream_edge_tts(text: str, voice: str, rate: str, audio_device: str) -> None:
    import edge_tts  # type: ignore

    # mpv is preferred because it accepts an explicit ALSA device, but the
    # recovery image may only contain ffplay. Keep TTS usable without root
    # package installation by selecting an available stdin MP3 player.
    mpv = shutil.which("mpv")
    ffplay = shutil.which("ffplay")
    if mpv:
        player_cmd = [mpv, "--no-video", "--really-quiet", "--cache=no", f"--audio-device={audio_device}", "-"]
    elif ffplay:
        player_cmd = [ffplay, "-nodisp", "-autoexit", "-loglevel", "error", "-f", "mp3", "-i", "-"]
    else:
        raise FileNotFoundError("neither mpv nor ffplay is installed")
    player = subprocess.Popen(player_cmd, stdin=subprocess.PIPE)
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
        engine = pyttsx3.init()
        engine.say(text)
        engine.runAndWait()
    except Exception as error:
        print(f"pyttsx3 failed: {type(error).__name__}: {error}", file=sys.stderr)
        return False
    return True


def log_doa_shadow(args: argparse.Namespace, tracker: AcousticDirectionTracker | None, end_time: float):
    if tracker is None:
        return None
    estimate = tracker.estimate(
        end_time=end_time,
        window_seconds=min(args.doa_window_seconds, args.doa_wake_window_seconds),
    )
    disposition = estimate.gimbal_disposition(-87.0, 87.0)
    angle = "none" if estimate.angle_deg is None else f"{estimate.angle_deg:.1f}deg"
    spread = "none" if estimate.circular_spread_deg is None else f"{estimate.circular_spread_deg:.1f}deg"
    print(
        f"DoA shadow> valid={estimate.valid} angle={angle} spread={spread} "
        f"valid_ratio={estimate.valid_ratio:.2f} samples={estimate.valid_sample_count}/"
        f"{estimate.sample_count} disposition={disposition} reason={estimate.reason}"
    )
    return estimate


def _run_turn(
    args: argparse.Namespace,
    policy: QwenChatPolicy,
    history: list[dict[str, str]],
    direction_tracker: AcousticDirectionTracker | None = None,
    face: ExpressionBridge | None = None,
) -> bool:
    turn_started = time.perf_counter()
    doa_window_end: float | None = None
    audio_started_at: float | None = None
    doa_estimate = None
    speaker_turn_done = False
    if args.text:
        user_text = args.text
        stt_elapsed = 0.0
        if face is not None:
            face.show("thinking")
    else:
        with tempfile.TemporaryDirectory() as tmp:
            audio_path = Path(args.audio_file) if args.audio_file else Path(tmp) / "turn.wav"
            if args.audio_file is None:
                if args.vad:
                    if not args.alsa_device:
                        raise RuntimeError("--vad requires --alsa-device")
                    speech_started_at, speech_ended_at = record_wav_with_vad_retry(
                        audio_path,
                        args.samplerate,
                        args.alsa_device,
                        args.vad_threshold,
                        args.vad_max_threshold,
                        args.vad_silence,
                        args.vad_max_seconds,
                        args.vad_channel,
                        on_speech_detected=(
                            lambda: face.show("listening") if face is not None else None
                        ),
                    )
                    doa_window_end = min(
                        speech_started_at + args.doa_window_seconds,
                        speech_ended_at,
                    )
                    with wave.open(str(audio_path), "rb") as recorded_wav:
                        audio_duration = recorded_wav.getnframes() / recorded_wav.getframerate()
                    audio_started_at = speech_ended_at - audio_duration
                elif args.alsa_device:
                    record_wav_with_arecord(audio_path, args.seconds, args.samplerate, args.alsa_device)
                else:
                    record_wav(audio_path, args.seconds, args.samplerate, args.input_device)
            if args.save_last_audio and args.audio_file is None:
                shutil.copyfile(audio_path, Path(args.save_last_audio).expanduser())
            if args.kws_model_dir and args.wake_word:
                if not args.kws_keywords_file:
                    raise RuntimeError("--kws-model-dir requires --kws-keywords-file")
                kws_started = time.perf_counter()
                keyword, keyword_start_s, keyword_end_s = detect_wake_word_audio_with_timing(
                    audio_path,
                    args.kws_model_dir,
                    args.kws_keywords_file,
                    args.kws_threads,
                    args.kws_score,
                    args.kws_threshold,
                )
                kws_elapsed = time.perf_counter() - kws_started
                if not keyword:
                    print(f"KWS> wake word not found; skipped full STT ({kws_elapsed:.2f}s)")
                    if face is not None:
                        face.idle()
                    return True
                if audio_started_at is not None and keyword_end_s is not None:
                    doa_window_end = min(audio_started_at + keyword_end_s + 0.12, speech_ended_at)
                timing = (
                    "unknown" if keyword_start_s is None or keyword_end_s is None
                    else f"{keyword_start_s:.2f}-{keyword_end_s:.2f}s"
                )
                print(
                    f"KWS> detected {keyword} at {timing} ({kws_elapsed:.2f}s); "
                    "running full STT"
                )
                if doa_window_end is not None:
                    doa_estimate = log_doa_shadow(args, direction_tracker, doa_window_end)
                    speaker_turn = execute_look_at_me(
                        doa_estimate.angle_deg if doa_estimate and doa_estimate.valid else None,
                        executable=args.gimbalctl,
                    )
                    speaker_turn_done = True
                    print(f"Speaker turn> {speaker_turn.reply}")
            print("Transcribing speech...")
            if face is not None:
                face.show("thinking")
            stt_started = time.perf_counter()
            user_text = transcribe_audio(audio_path, args.paraformer_model)
            stt_elapsed = time.perf_counter() - stt_started

    if not user_text:
        print("STT> no speech detected")
        return True
    if user_text.lower() in {"q", "quit", "exit"}:
        return False

    transcribed_text = user_text
    user_text = extract_wake_command(transcribed_text, args.wake_word)
    if user_text is None:
        print(f"Ignored> wake word not found: {transcribed_text}")
        return True
    if doa_window_end is not None and not speaker_turn_done:
        doa_estimate = log_doa_shadow(args, direction_tracker, doa_window_end)
        speaker_turn = execute_look_at_me(
            doa_estimate.angle_deg if doa_estimate and doa_estimate.valid else None,
            executable=args.gimbalctl,
        )
        print(f"Speaker turn> {speaker_turn.reply}")
    if not user_text:
        print("Wake word detected, but no command followed.")
        if face is not None:
            face.show("wake")
            if not args.no_tts:
                face.show("speaking")
        speak("我在", enabled=not args.no_tts)
        return True

    print(f"你> {user_text}")
    if "看我" in "".join(user_text.split()):
        detector = TargetDetector(state_url=args.perception_state_url,
                                  detections_url=args.yolo_detections_url)
        time.sleep(1.0)
        person = detector.detect("person")
        if person is None:
            reply = "未识别到人物，保持当前声源方向"
        elif person.box and person.image_width and person.image_height:
            reply = refine_person_alignment(person.box, person.image_width, person.image_height,
                                            executable=args.gimbalctl).reply
        else:
            reply = "已保持当前声源方向"
        print(f"Gimbal> {reply}")
        speak(reply, enabled=not args.no_tts)
        history.append({"role": "user", "content": user_text})
        history.append({"role": "assistant", "content": reply})
        return True
    intent = parse_user_intent(user_text)
    gimbal_result = execute_gimbal_voice_command(
        user_text,
        executable=args.gimbalctl,
        default_step_deg=args.gimbal_step_deg,
    )
    if gimbal_result.recognized:
        if face is not None:
            face.show("gimbal")
        print(f"Gimbal> {gimbal_result.reply}")
        if not args.no_tts:
            print("Generating speech...")
        tts_started = time.perf_counter()
        speak(gimbal_result.reply, enabled=not args.no_tts)
        if face is not None:
            # A setting/action result remains visible until the next voice state.
            # Avoiding a short speaking transition also removes two screen writes.
            face.keep_current()
        tts_elapsed = time.perf_counter() - tts_started
        print(
            f"Timing> STT {stt_elapsed:.2f}s | local gimbal | "
            f"TTS {tts_elapsed:.2f}s | turn {time.perf_counter() - turn_started:.2f}s"
        )
        history.append({"role": "user", "content": user_text})
        history.append({"role": "assistant", "content": gimbal_result.reply})
        return True

    visual_request = should_use_camera(user_text, intent)
    if face is not None and visual_request:
        face.show("look")
    perception_started = time.perf_counter()
    perception = TargetDetector(
        state_url=args.perception_state_url,
        detections_url=args.yolo_detections_url,
    ).answer(user_text)
    if perception.recognized:
        if face is not None:
            face.show("look")
        perception_elapsed = time.perf_counter() - perception_started
        print(f"Perception> {perception.reply}")
        if not args.no_tts:
            print("Generating speech...")
        tts_started = time.perf_counter()
        if face is not None and not args.no_tts:
            face.show("speaking")
        speak(perception.reply, enabled=not args.no_tts)
        tts_elapsed = time.perf_counter() - tts_started
        print(
            f"Timing> STT {stt_elapsed:.2f}s | YOLO+depth {perception_elapsed:.2f}s | "
            f"TTS {tts_elapsed:.2f}s | turn {time.perf_counter() - turn_started:.2f}s"
        )
        history.append({"role": "user", "content": user_text})
        history.append({"role": "assistant", "content": perception.reply})
        return True

    image_path = None
    if args.camera_bridge_dir and visual_request:
        print("Capturing camera image...")
        image_path = request_camera_image(args.camera_bridge_dir)
    print("Waiting for AI response...")
    if face is not None and not visual_request:
        face.show("thinking")
    llm_started = time.perf_counter()
    decision = policy.decide_from_user(
        user_text, DEMO_STATE, image_path=image_path, history=history, intent=intent
    )
    llm_elapsed = time.perf_counter() - llm_started
    print(f"AI> {decision.reply}")
    print(f"任务预览> {decision.task.type if decision.task else decision.action} | 安全> {decision.safety} | 原因> {decision.reason}")
    if not args.no_tts:
        print("Generating speech...")
    tts_started = time.perf_counter()
    if face is not None and not args.no_tts:
        face.show("speaking")
    speak(decision.reply, enabled=not args.no_tts)
    tts_elapsed = time.perf_counter() - tts_started
    processing_elapsed = stt_elapsed + llm_elapsed + tts_elapsed
    print(
        f"Timing> STT {stt_elapsed:.2f}s | API {llm_elapsed:.2f}s | "
        f"TTS {tts_elapsed:.2f}s | processing {processing_elapsed:.2f}s | "
        f"turn {time.perf_counter() - turn_started:.2f}s"
    )
    history.append({"role": "user", "content": user_text})
    history.append({"role": "assistant", "content": decision.reply})
    return True


def run_turn(
    args: argparse.Namespace,
    policy: QwenChatPolicy,
    history: list[dict[str, str]],
    direction_tracker: AcousticDirectionTracker | None = None,
    face: ExpressionBridge | None = None,
) -> bool:
    failed = False
    try:
        return _run_turn(args, policy, history, direction_tracker, face)
    except Exception:
        failed = True
        if face is not None:
            face.show("error", duration_ms=2000)
        raise
    finally:
        if face is not None and not failed:
            face.idle()


def warm_up_stt(args: argparse.Namespace) -> None:
    """Load and warm the configured local STT model before listening."""
    print("Warming up Paraformer...")
    started = time.perf_counter()
    model = load_paraformer_model(args.paraformer_model)
    with tempfile.TemporaryDirectory() as tmp:
        warmup_path = Path(tmp) / "warmup.wav"
        with wave.open(str(warmup_path), "wb") as wav:
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(args.samplerate)
            wav.writeframes(bytes(args.samplerate * 2))
        model([str(warmup_path)])
    print(f"Paraformer ready in {time.perf_counter() - started:.2f}s.")


def warm_up_kws(args: argparse.Namespace) -> None:
    if not args.kws_model_dir:
        return
    if not args.kws_keywords_file:
        raise RuntimeError("--kws-model-dir requires --kws-keywords-file")
    print("Warming up keyword spotter...")
    started = time.perf_counter()
    load_keyword_spotter(
        args.kws_model_dir,
        args.kws_keywords_file,
        max(1, args.kws_threads),
        args.kws_score,
        args.kws_threshold,
    )
    print(f"Keyword spotter ready in {time.perf_counter() - started:.2f}s.")


def main() -> int:
    args = build_parser().parse_args()
    face = None
    if args.face_map:
        try:
            face = ExpressionBridge.from_file(args.face_map, socket_path=args.face_socket)
            face.set_default("idle")
            face.idle(force=True)
            print(f"Face expressions enabled: {args.face_map}")
        except (OSError, ValueError) as error:
            print(f"Face expressions disabled: {error}", file=sys.stderr)
    if not os.getenv("DASHSCOPE_API_KEY"):
        print("DASHSCOPE_API_KEY is not set.")
        return 2
    try:
        policy = QwenChatPolicy(timeout=args.llm_timeout)
    except QwenPolicyError as exc:
        print(exc)
        return 2

    print("Voice chat demo. It will not publish /cmd_vel or control the robot.")
    try:
        warm_up_kws(args)
    except Exception as error:
        print(f"Keyword spotter warmup failed: {error}", file=sys.stderr)
        return 2
    try:
        warm_up_stt(args)
    except Exception as error:
        print(f"Paraformer warmup failed; continuing without warmup: {error}")
    history: list[dict[str, str]] = []
    direction_tracker = None
    if args.doa_shadow:
        direction_tracker = AcousticDirectionTracker(
            args.doa_state_file,
            # Preserve the VAD window while Paraformer and wake-word matching run.
            history_seconds=max(90.0, args.vad_max_seconds + args.doa_window_seconds + 10.0),
        )
        direction_tracker.start()
        print(f"DoA shadow enabled: {args.doa_state_file}")
    try:
        while True:
            if args.audio_file is None and args.text is None and not args.vad:
                answer = input("Press Enter to record, or q to quit: ").strip().lower()
                if answer in {"q", "quit", "exit"}:
                    return 0
            if not run_turn(args, policy, history, direction_tracker, face):
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
    finally:
        if direction_tracker is not None:
            direction_tracker.close()


if __name__ == "__main__":
    raise SystemExit(main())
