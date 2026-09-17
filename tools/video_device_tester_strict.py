#!/usr/bin/env python3

from __future__ import annotations

import argparse
import ctypes
import glob
import json
import math
import os
import platform
import queue
import re
import shutil
import statistics
import struct
import subprocess
import sys
import threading
import time
import uuid
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Optional

try:
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk
except ImportError as exc:
    raise SystemExit("缺少 Tkinter。Linux 请安装发行版提供的 python3-tk。") from exc


APP_NAME = "严格视频设备能力与实测工作台"
APP_VERSION = "3.5.0"
STANDARD_FPS = (1.0, 5.0, 10.0, 15.0, 20.0, 24.0, 25.0, 29.97, 30.0, 50.0, 59.94, 60.0, 90.0, 120.0)
COMMON_SIZES = ((320, 240), (640, 480), (800, 600), (1024, 768), (1280, 720),
                (1920, 1080), (2560, 1440), (3840, 2160))


@dataclass
class Device:
    id: str
    label: str
    backend: str
    source: Any
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class Mode:
    format_label: str
    input_format: str
    width: int
    height: int
    fps: float
    fps_kind: str
    option_kind: str
    source: str
    evidence: str = ""

    def key(self) -> tuple:
        return (self.format_label, self.input_format, self.width, self.height, round(self.fps, 6), self.option_kind)

    def label(self) -> str:
        return f"{self.format_label} · {self.width}×{self.height} @ {self.fps:g} FPS"


def now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def mean(values: list[float]) -> Optional[float]:
    return statistics.fmean(values) if values else None


def percentile(values: list[float], p: float) -> Optional[float]:
    if not values:
        return None
    data = sorted(values)
    if len(data) == 1:
        return float(data[0])
    position = (len(data) - 1) * p
    low, high = math.floor(position), math.ceil(position)
    if low == high:
        return float(data[low])
    return float(data[low] * (high - position) + data[high] * (position - low))


def rounded(value: Any, digits: int = 3) -> Any:
    return round(float(value), digits) if value is not None else None


def decode_bytes(data: bytes) -> str:
    for encoding in ("utf-8-sig", "gb18030", sys.getfilesystemencoding()):
        if not encoding:
            continue
        try:
            return data.decode(encoding)
        except (UnicodeDecodeError, LookupError):
            pass
    return data.decode("utf-8", errors="replace")


def run_command(args: list[str], timeout: float = 15.0) -> tuple[int, str]:
    try:
        result = subprocess.run(
            args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout,
            creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
        )
        output = decode_bytes(result.stdout)
        error = decode_bytes(result.stderr)
        return result.returncode, output + (("\n" + error) if error else "")
    except (OSError, subprocess.TimeoutExpired) as exc:
        return -1, str(exc)


def locate_ffmpeg(explicit: str = "") -> Optional[str]:
    script_dir = Path(__file__).resolve().parent
    exe = "ffmpeg.exe" if os.name == "nt" else "ffmpeg"
    system_key = {"Windows": "windows", "Linux": "linux", "Darwin": "macos"}.get(platform.system(), platform.system().lower())
    candidates = [
        explicit,
        str(script_dir / exe),
        str(script_dir / "bin" / exe),
        str(script_dir / "tools" / system_key / exe),
        str(Path(sys.prefix) / "Library" / "bin" / "ffmpeg.exe"),
        str(Path(sys.prefix) / "bin" / "ffmpeg"),
        shutil.which("ffmpeg") or "",
    ]
    for candidate in dict.fromkeys(filter(None, candidates)):
        path = Path(candidate).expanduser()
        if path.is_file():
            code, output = run_command([str(path), "-version"], 5)
            if code == 0 and "ffmpeg version" in output.lower():
                return str(path.resolve())
    return None


def locate_ffplay(ffmpeg: str = "") -> Optional[str]:
    script_dir = Path(__file__).resolve().parent
    exe = "ffplay.exe" if os.name == "nt" else "ffplay"
    system_key = {"Windows": "windows", "Linux": "linux", "Darwin": "macos"}.get(platform.system(), platform.system().lower())
    beside_ffmpeg = str(Path(ffmpeg).resolve().with_name(exe)) if ffmpeg else ""
    candidates = [
        beside_ffmpeg,
        str(script_dir / exe),
        str(script_dir / "bin" / exe),
        str(script_dir / "tools" / system_key / exe),
        str(Path(sys.prefix) / "Library" / "bin" / "ffplay.exe"),
        str(Path(sys.prefix) / "bin" / "ffplay"),
        shutil.which("ffplay") or "",
    ]
    for candidate in dict.fromkeys(filter(None, candidates)):
        path = Path(candidate).expanduser()
        if path.is_file():
            code, output = run_command([str(path), "-version"], 5)
            if code == 0 and "ffplay version" in output.lower():
                return str(path.resolve())
    return None


def ffmpeg_version(ffmpeg: str) -> str:
    _, output = run_command([ffmpeg, "-version"], 5)
    return output.splitlines()[0] if output else "unknown"


def _expand_fps_range(minimum: float, maximum: float) -> list[float]:
    low, high = sorted((minimum, maximum))
    values = [low, high]
    values.extend(value for value in STANDARD_FPS if low - 0.02 <= value <= high + 0.02)
    return sorted({round(value, 6) for value in values if value > 0})


def _dedupe_modes(modes: list[Mode]) -> list[Mode]:
    result: list[Mode] = []
    seen = set()
    for mode in modes:
        if mode.key() not in seen:
            result.append(mode)
            seen.add(mode.key())
    return sorted(result, key=lambda m: (m.format_label, m.width * m.height, m.width, m.fps))


def enumerate_windows(ffmpeg: str) -> tuple[list[Device], str]:
    _, output = run_command([ffmpeg, "-hide_banner", "-list_devices", "true", "-f", "dshow", "-i", "dummy"], 15)
    devices: list[Device] = []
    counts: dict[str, int] = {}
    lines = output.splitlines()
    for index, line in enumerate(lines):
        match = re.search(r'"(.+?)"\s+\(video\)\s*$', line.strip(), re.I)
        if not match:
            continue
        name = match.group(1)
        number = counts.get(name, 0)
        counts[name] = number + 1
        alternate = ""
        if index + 1 < len(lines):
            alt_match = re.search(r'Alternative name\s+"(.+?)"', lines[index + 1], re.I)
            if alt_match:
                alternate = alt_match.group(1)
        suffix = f" #{number + 1}" if number else ""
        devices.append(Device(
            id=f"dshow:{name}:{number}", label=name + suffix, backend="dshow",
            source={"name": name, "number": number, "alternate_name": alternate},
            metadata={"friendly_name": name, "alternate_name": alternate, "device_number": number},
        ))
    return devices, output


def enumerate_linux(_ffmpeg: str) -> tuple[list[Device], str]:
    devices: list[Device] = []
    raw_lines = []
    for value in sorted(glob.glob("/dev/video*"), key=lambda x: int(re.search(r"(\d+)$", x).group(1)) if re.search(r"(\d+)$", x) else 9999):
        path = Path(value)
        sys_node = Path("/sys/class/video4linux") / path.name
        name = ""
        try:
            name = (sys_node / "name").read_text(errors="replace").strip()
        except OSError:
            pass
        metadata: dict[str, Any] = {"path": value, "sysfs": str(sys_node)}
        try:
            resolved = (sys_node / "device").resolve()
            metadata["device_path"] = str(resolved)
            current = resolved
            for _ in range(8):
                for key in ("idVendor", "idProduct", "serial", "manufacturer", "product", "busnum", "devnum"):
                    target = current / key
                    if key not in metadata and target.is_file():
                        text = target.read_text(errors="replace").strip()
                        if text:
                            metadata[key] = text
                if current.parent == current:
                    break
                current = current.parent
            try:
                metadata["driver"] = (resolved / "driver").resolve().name
            except OSError:
                pass
        except OSError:
            pass
        devices.append(Device(value, f"{value} — {name or '视频设备'}", "v4l2", value, metadata))
        raw_lines.append(json.dumps(metadata, ensure_ascii=False))
    return devices, "\n".join(raw_lines)


def enumerate_macos(ffmpeg: str) -> tuple[list[Device], str]:
    _, output = run_command([ffmpeg, "-hide_banner", "-f", "avfoundation", "-list_devices", "true", "-i", ""], 15)
    devices: list[Device] = []
    in_video = False
    for line in output.splitlines():
        if "AVFoundation video devices" in line:
            in_video = True
            continue
        if "AVFoundation audio devices" in line:
            in_video = False
            continue
        if not in_video:
            continue
        match = re.search(r"\[(\d+)\]\s+(.+?)\s*$", line)
        if match:
            index, name = int(match.group(1)), match.group(2).strip()
            devices.append(Device(f"avfoundation:{index}", name, "avfoundation", index, {"index": index}))
    return devices, output


def enumerate_devices(ffmpeg: str) -> tuple[list[Device], str]:
    system = platform.system()
    if system == "Windows":
        return enumerate_windows(ffmpeg)
    if system == "Linux":
        return enumerate_linux(ffmpeg)
    if system == "Darwin":
        return enumerate_macos(ffmpeg)
    raise RuntimeError(f"暂不支持操作系统：{system}")


def parse_dshow_modes(text: str) -> list[Mode]:
    modes: list[Mode] = []
    pattern = re.compile(
        r"(?P<kind>pixel_format|vcodec)=(?P<fmt>[^\s]+).*?"
        r"min\s+s=(?P<minw>\d+)x(?P<minh>\d+)\s+fps=(?P<minfps>[0-9.]+).*?"
        r"max\s+s=(?P<maxw>\d+)x(?P<maxh>\d+)\s+fps=(?P<maxfps>[0-9.]+)", re.I,
    )
    for line in text.splitlines():
        match = pattern.search(line)
        if not match:
            continue
        option_kind = match.group("kind").lower()
        input_format = match.group("fmt").lower()
        format_label = input_format.upper().replace("YUYV422", "YUYV").replace("MJPEG", "MJPG")
        endpoints = {
            (int(match.group("minw")), int(match.group("minh"))),
            (int(match.group("maxw")), int(match.group("maxh"))),
        }
        minimum, maximum = float(match.group("minfps")), float(match.group("maxfps"))
        evidence = line.strip()
        for width, height in endpoints:
            for fps in _expand_fps_range(minimum, maximum):
                modes.append(Mode(format_label, input_format, width, height, fps,
                                  "range" if minimum != maximum else "discrete",
                                  option_kind, "FFmpeg DirectShow list_options", evidence))
    return _dedupe_modes(modes)


# Linux V4L2 ioctl definitions. These match linux/videodev2.h fixed-width layouts.
def _ioc(direction: int, type_char: str, number: int, size: int) -> int:
    return (direction << 30) | (size << 16) | (ord(type_char) << 8) | number


VIDIOC_ENUM_FMT = _ioc(3, "V", 2, 64)
VIDIOC_ENUM_FRAMESIZES = _ioc(3, "V", 74, 44)
VIDIOC_ENUM_FRAMEINTERVALS = _ioc(3, "V", 75, 52)
V4L2_BUF_TYPE_VIDEO_CAPTURE = 1


def _fourcc(number: int) -> str:
    return "".join(chr((number >> (8 * index)) & 0xFF) for index in range(4)).rstrip("\x00 ")


def _v4l2_ioctl(fd: int, request: int, buffer: bytearray) -> bool:
    import fcntl  # Linux-only standard library module.
    try:
        fcntl.ioctl(fd, request, buffer, True)
        return True
    except OSError:
        return False


def linux_v4l2_modes(path: str) -> tuple[list[Mode], str]:
    modes: list[Mode] = []
    raw: list[str] = []
    fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
    try:
        format_index = 0
        while True:
            fmt_buffer = bytearray(64)
            struct.pack_into("II", fmt_buffer, 0, format_index, V4L2_BUF_TYPE_VIDEO_CAPTURE)
            if not _v4l2_ioctl(fd, VIDIOC_ENUM_FMT, fmt_buffer):
                break
            flags = struct.unpack_from("I", fmt_buffer, 8)[0]
            description = bytes(fmt_buffer[12:44]).split(b"\0", 1)[0].decode(errors="replace")
            pixel_format = struct.unpack_from("I", fmt_buffer, 44)[0]
            fourcc = _fourcc(pixel_format)
            raw.append(f"FORMAT {format_index}: {fourcc} ({description}), flags=0x{flags:x}")
            size_index = 0
            while True:
                size_buffer = bytearray(44)
                struct.pack_into("II", size_buffer, 0, size_index, pixel_format)
                if not _v4l2_ioctl(fd, VIDIOC_ENUM_FRAMESIZES, size_buffer):
                    break
                size_type = struct.unpack_from("I", size_buffer, 8)[0]
                sizes: list[tuple[int, int, str]] = []
                if size_type == 1:
                    width, height = struct.unpack_from("II", size_buffer, 12)
                    sizes.append((width, height, "discrete"))
                else:
                    min_w, max_w, step_w, min_h, max_h, step_h = struct.unpack_from("IIIIII", size_buffer, 12)
                    sizes.extend([(min_w, min_h, "range-min"), (max_w, max_h, "range-max")])
                    raw.append(f"  SIZE RANGE {min_w}x{min_h} – {max_w}x{max_h}, step={step_w}x{step_h}")
                for width, height, size_kind in sizes:
                    interval_index = 0
                    found_interval = False
                    while True:
                        interval_buffer = bytearray(52)
                        struct.pack_into("IIII", interval_buffer, 0, interval_index, pixel_format, width, height)
                        if not _v4l2_ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, interval_buffer):
                            break
                        found_interval = True
                        interval_type = struct.unpack_from("I", interval_buffer, 16)[0]
                        evidence = ""
                        if interval_type == 1:
                            numerator, denominator = struct.unpack_from("II", interval_buffer, 20)
                            fps_values = [denominator / numerator] if numerator else []
                            fps_kind = "discrete"
                            evidence = f"{numerator}/{denominator}s"
                        else:
                            min_num, min_den, max_num, max_den, step_num, step_den = struct.unpack_from("IIIIII", interval_buffer, 20)
                            max_fps = min_den / min_num if min_num else 0
                            min_fps = max_den / max_num if max_num else 0
                            fps_values = _expand_fps_range(min_fps, max_fps)
                            fps_kind = "range"
                            evidence = f"interval {min_num}/{min_den} – {max_num}/{max_den}, step {step_num}/{step_den}"
                        for fps in fps_values:
                            modes.append(Mode(fourcc, fourcc, width, height, fps, fps_kind,
                                              "input_format", "V4L2 VIDIOC_ENUM_*", evidence))
                            raw.append(f"  {fourcc} {width}x{height} @{fps:g} ({fps_kind}, {size_kind})")
                        interval_index += 1
                    if not found_interval:
                        raw.append(f"  {fourcc} {width}x{height}: 未枚举到帧间隔")
                size_index += 1
            format_index += 1
    finally:
        os.close(fd)
    return _dedupe_modes(modes), "\n".join(raw)


def parse_avfoundation_modes(text: str) -> list[Mode]:
    modes: list[Mode] = []
    pixel_formats: list[str] = []
    for line in text.splitlines():
        pixel_match = re.search(r"Supported pixel formats?:\s*(.+)", line, re.I)
        if pixel_match:
            pixel_formats = [item.strip().strip("',") for item in re.split(r"[, ]+", pixel_match.group(1)) if item.strip()]
        mode_match = re.search(r"(\d+)x(\d+)@\[([0-9.]+)\s+([0-9.]+)\]fps", line)
        if not mode_match:
            continue
        width, height = int(mode_match.group(1)), int(mode_match.group(2))
        minimum, maximum = float(mode_match.group(3)), float(mode_match.group(4))
        formats = pixel_formats or ["auto"]
        for fmt in formats:
            for fps in _expand_fps_range(minimum, maximum):
                modes.append(Mode(fmt.upper(), fmt, width, height, fps, "range", "pixel_format",
                                  "FFmpeg AVFoundation supported modes", line.strip()))
    return _dedupe_modes(modes)


def read_capabilities(ffmpeg: str, device: Device) -> tuple[list[Mode], str]:
    if device.backend == "dshow":
        args = [ffmpeg, "-hide_banner", "-list_options", "true", "-f", "dshow"]
        if device.source.get("number", 0):
            args.extend(["-video_device_number", str(device.source["number"])])
        args.extend(["-i", "video=" + device.source["name"]])
        _, output = run_command(args, 20)
        return parse_dshow_modes(output), output
    if device.backend == "v4l2":
        return linux_v4l2_modes(str(device.source))
    if device.backend == "avfoundation":
        args = [ffmpeg, "-hide_banner", "-f", "avfoundation", "-framerate", "999",
                "-video_size", "1x1", "-i", f"{device.source}:none", "-t", "0.1", "-f", "null", "-"]
        _, output = run_command(args, 20)
        return parse_avfoundation_modes(output), output
    raise RuntimeError(f"未知后端：{device.backend}")


def linux_input_format(fourcc: str) -> str:
    return {
        "MJPG": "mjpeg", "JPEG": "mjpeg", "YUYV": "yuyv422", "YUY2": "yuyv422",
        "NV12": "nv12", "H264": "h264", "HEVC": "hevc", "H265": "hevc",
        "RGB3": "rgb24", "BGR3": "bgr24", "GREY": "gray", "Y16 ": "gray16le",
    }.get(fourcc.upper(), fourcc.lower())


def input_args(device: Device, mode: Mode) -> list[str]:
    size = f"{mode.width}x{mode.height}"
    fps = f"{mode.fps:.8g}"
    if device.backend == "dshow":
        args = ["-f", "dshow"]
        if device.source.get("number", 0):
            args.extend(["-video_device_number", str(device.source["number"])])
        if mode.option_kind == "vcodec":
            args.extend(["-vcodec", mode.input_format])
        elif mode.input_format and mode.input_format != "auto":
            args.extend(["-pixel_format", mode.input_format])
        args.extend(["-video_size", size, "-framerate", fps, "-i", "video=" + device.source["name"]])
        return args
    if device.backend == "v4l2":
        return ["-f", "v4l2", "-input_format", linux_input_format(mode.input_format),
                "-video_size", size, "-framerate", fps, "-i", str(device.source)]
    if device.backend == "avfoundation":
        args = ["-f", "avfoundation"]
        if mode.input_format and mode.input_format != "auto":
            args.extend(["-pixel_format", mode.input_format])
        args.extend(["-video_size", size, "-framerate", fps, "-i", f"{device.source}:none"])
        return args
    raise RuntimeError(f"未知后端：{device.backend}")


SHOWINFO_RE = re.compile(
    r"showinfo.*?\bn:\s*(\d+).*?\bpts_time:([\-0-9.eE]+).*?\bfmt:([^\s]+).*?\bs:(\d+)x(\d+)", re.I
)


def parse_test_output(stdout: str, stderr: str, requested: Mode, wall_s: float, returncode: int,
                      warmup: float, duration: float, stopped: bool) -> dict[str, Any]:
    progress: dict[str, str] = {}
    for line in stdout.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            progress[key.strip()] = value.strip()
    frames = []
    for line in stderr.splitlines():
        match = SHOWINFO_RE.search(line)
        if match:
            frames.append({
                "n": int(match.group(1)), "pts_time": float(match.group(2)),
                "decoded_format": match.group(3), "width": int(match.group(4)), "height": int(match.group(5)),
            })
    timestamps = [item["pts_time"] for item in frames]
    intervals_ms = [(b - a) * 1000.0 for a, b in zip(timestamps, timestamps[1:]) if b >= a]
    media_span = timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0.0
    measured_fps = (len(timestamps) - 1) / media_span if media_span > 0 else 0.0
    expected_ms = 1000.0 / requested.fps
    estimated_drops = sum(max(0, round(value / expected_ms) - 1) for value in intervals_ms)
    actual_width = frames[0]["width"] if frames else None
    actual_height = frames[0]["height"] if frames else None
    resolution_match = actual_width == requested.width and actual_height == requested.height
    fps_tolerance = max(0.5, requested.fps * 0.05)
    fps_match = measured_fps > 0 and abs(measured_fps - requested.fps) <= fps_tolerance
    enough_frames = len(frames) >= max(2, math.floor(duration * requested.fps * 0.75))
    passed = returncode == 0 and not stopped and resolution_match and fps_match and enough_frames
    failure_reasons = []
    if returncode != 0:
        failure_reasons.append(f"FFmpeg 退出码 {returncode}")
    if not frames:
        failure_reasons.append("没有 showinfo 帧证据")
    if frames and not resolution_match:
        failure_reasons.append(f"实际分辨率 {actual_width}x{actual_height} 与请求不符")
    if measured_fps and not fps_match:
        failure_reasons.append(f"实测 FPS {measured_fps:.3f} 与请求 {requested.fps:g} 偏差超限")
    if frames and not enough_frames:
        failure_reasons.append("成功帧数低于请求值的 75%")
    if stopped:
        failure_reasons.append("用户停止")
    return {
        "status": "PASS" if passed else ("STOPPED" if stopped else "FAIL"),
        "requested": asdict(requested),
        "actual": {
            "width": actual_width, "height": actual_height,
            "decoded_pixel_format": frames[0]["decoded_format"] if frames else None,
        },
        "measurement": {
            "warmup_s": warmup, "requested_duration_s": duration, "wall_elapsed_s": rounded(wall_s),
            "showinfo_frames": len(frames), "media_time_span_s": rounded(media_span),
            "measured_fps_from_pts": rounded(measured_fps),
            "fps_error_percent": rounded((measured_fps / requested.fps - 1) * 100.0, 2) if measured_fps else None,
            "frame_interval_mean_ms": rounded(mean(intervals_ms)),
            "frame_interval_p50_ms": rounded(percentile(intervals_ms, 0.50)),
            "frame_interval_p95_ms": rounded(percentile(intervals_ms, 0.95)),
            "frame_interval_max_ms": rounded(max(intervals_ms) if intervals_ms else None),
            "frame_interval_jitter_std_ms": rounded(statistics.pstdev(intervals_ms) if len(intervals_ms) > 1 else 0.0),
            "long_interval_count": sum(value > expected_ms * 1.5 for value in intervals_ms),
            "estimated_dropped_frames_from_pts": int(estimated_drops),
            "ffmpeg_drop_frames": int(progress.get("drop_frames", "0") or 0),
            "ffmpeg_dup_frames": int(progress.get("dup_frames", "0") or 0),
            "ffmpeg_reported_fps": progress.get("fps"), "ffmpeg_speed": progress.get("speed"),
            "ffmpeg_out_time": progress.get("out_time"),
        },
        "strict_checks": {
            "ffmpeg_exit_ok": returncode == 0, "resolution_match": resolution_match,
            "fps_within_tolerance": fps_match, "enough_frames_75_percent": enough_frames,
            "fps_tolerance": fps_tolerance,
        },
        "failure_reasons": failure_reasons,
        "ffmpeg_progress": progress,
        "returncode": returncode,
        "raw_stdout": stdout,
        "raw_stderr": stderr,
    }


def build_test_command(ffmpeg: str, device: Device, mode: Mode, warmup: float, duration: float) -> list[str]:
    command = [ffmpeg, "-hide_banner", "-nostdin", "-loglevel", "info", "-stats_period", "0.5"]
    command.extend(input_args(device, mode))
    video_filter = f"trim=start={warmup:.6f}:duration={duration:.6f},setpts=PTS-STARTPTS,showinfo"
    command.extend(["-map", "0:v:0", "-an", "-vf", video_filter, "-t", f"{duration:.6f}",
                    "-f", "null", "-", "-progress", "pipe:1", "-nostats"])
    return command


def build_native_preview_command(ffmpeg: str, device: Device, mode: Mode, duration: float) -> list[str]:
    command = [ffmpeg, "-hide_banner", "-nostdin", "-loglevel", "warning"]
    command.extend(input_args(device, mode))
    command.extend([
        "-map", "0:v:0", "-an", "-t", f"{duration:.6f}",
        "-c:v", "copy", "-f", "nut", "pipe:1",
    ])
    return command


def run_native_preview(ffmpeg: str, ffplay: str, device: Device, mode: Mode, duration: float,
                       stop_event: threading.Event,
                       process_callback: Callable[[Optional[subprocess.Popen]], None],
                       progress_callback: Callable[[float], None]) -> dict[str, Any]:
    command = build_native_preview_command(ffmpeg, device, mode, duration)
    viewer_command = [
        ffplay, "-hide_banner", "-loglevel", "warning", "-nostats", "-autoexit",
        "-framedrop", "-sync", "video", "-window_title", mode.label(), "-i", "pipe:0",
    ]
    creationflags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
    viewer = subprocess.Popen(
        viewer_command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
        creationflags=creationflags,
    )
    try:
        process = subprocess.Popen(
            command, stdout=viewer.stdin, stderr=subprocess.PIPE,
            stdin=subprocess.DEVNULL, creationflags=creationflags,
        )
    except Exception:
        if viewer.poll() is None:
            viewer.terminate()
        raise
    if viewer.stdin:
        viewer.stdin.close()
    process_callback(process)
    producer_stderr: list[bytes] = []
    viewer_stderr: list[bytes] = []

    def drain(stream, target):
        while True:
            chunk = stream.read(65536)
            if not chunk:
                break
            target.append(chunk)

    threads = [
        threading.Thread(target=drain, args=(process.stderr, producer_stderr), daemon=True),
        threading.Thread(target=drain, args=(viewer.stderr, viewer_stderr), daemon=True),
    ]
    for thread in threads:
        thread.start()
    started = time.perf_counter()
    stopped = False
    while process.poll() is None:
        elapsed = time.perf_counter() - started
        progress_callback(min(99.0, elapsed / max(duration, 0.1) * 100.0))
        if stop_event.is_set():
            stopped = True
            process.terminate()
            break
        if elapsed > duration + 15.0:
            process.kill()
            break
        time.sleep(0.1)
    try:
        producer_returncode = process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill(); producer_returncode = process.wait()
    try:
        viewer_returncode = viewer.wait(timeout=5)
    except subprocess.TimeoutExpired:
        viewer.terminate()
        try:
            viewer_returncode = viewer.wait(timeout=2)
        except subprocess.TimeoutExpired:
            viewer.kill(); viewer_returncode = viewer.wait()
    for thread in threads:
        thread.join(2)
    process_callback(None)
    return {
        "status": "STOPPED" if stopped else ("OK" if producer_returncode == 0 and viewer_returncode == 0 else "ERROR"),
        "duration_s": duration,
        "delivery": "original captured stream copied without video transcoding",
        "overlaps_strict_measurement": False,
        "producer_command": command,
        "viewer_command": viewer_command,
        "producer_returncode": producer_returncode,
        "viewer_returncode": viewer_returncode,
        "producer_stderr": decode_bytes(b"".join(producer_stderr)),
        "viewer_stderr": decode_bytes(b"".join(viewer_stderr)),
    }


def run_strict_test(ffmpeg: str, device: Device, mode: Mode, warmup: float, duration: float,
                    stop_event: threading.Event, process_callback: Callable[[Optional[subprocess.Popen]], None],
                    progress_callback: Callable[[float], None]) -> dict[str, Any]:
    command = build_test_command(ffmpeg, device, mode, warmup, duration)
    creationflags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               stdin=subprocess.DEVNULL, creationflags=creationflags)
    process_callback(process)
    stdout_parts: list[bytes] = []
    stderr_parts: list[bytes] = []

    def drain(stream, target):
        while True:
            chunk = stream.read(65536)
            if not chunk:
                break
            target.append(chunk)

    threads: list[threading.Thread] = []
    out_thread = threading.Thread(target=drain, args=(process.stdout, stdout_parts), daemon=True)
    threads.append(out_thread)
    err_thread = threading.Thread(target=drain, args=(process.stderr, stderr_parts), daemon=True)
    threads.append(err_thread)
    for thread in threads:
        thread.start()
    started = time.perf_counter()
    total_expected = warmup + duration
    stopped = False
    timeout = total_expected + 20.0
    while process.poll() is None:
        elapsed = time.perf_counter() - started
        progress_callback(min(99.0, elapsed / max(total_expected, 0.1) * 100.0))
        if stop_event.is_set():
            stopped = True
            process.terminate()
            break
        if elapsed > timeout:
            process.kill()
            break
        time.sleep(0.1)
    try:
        returncode = process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill(); returncode = process.wait()
    for thread in threads:
        thread.join(2)
    process_callback(None)
    wall_s = time.perf_counter() - started
    stderr_text = decode_bytes(b"".join(stderr_parts))
    stdout_text = decode_bytes(b"".join(stdout_parts))
    result = parse_test_output(stdout_text, stderr_text,
                               mode, wall_s, returncode, warmup, duration, stopped)
    result["command"] = command
    return result


class StrictTesterApp:
    def __init__(self, root: tk.Tk, initial_ffmpeg: str = ""):
        self.root = root
        self.root.title(f"{APP_NAME} v{APP_VERSION}")
        self.root.geometry("1260x820")
        self.root.minsize(1000, 680)
        self.events: queue.Queue = queue.Queue()
        self.devices: list[Device] = []
        self.modes: list[Mode] = []
        self.mode_by_item: dict[str, Mode] = {}
        self.plan: list[Mode] = []
        self.plan_by_item: dict[str, Mode] = {}
        self.results: list[dict[str, Any]] = []
        self.session_id = uuid.uuid4().hex
        self.session_started_at = now_iso()
        self.session_device_id: Optional[str] = None
        self.session_device_snapshot: Optional[Device] = None
        self._device_change_guard = False
        self.capability_raw = ""
        self.enumeration_raw = ""
        self.stop_event = threading.Event()
        self.worker: Optional[threading.Thread] = None
        self.current_process: Optional[subprocess.Popen] = None
        self.ffmpeg_var = tk.StringVar(value=initial_ffmpeg or locate_ffmpeg() or "")
        self.device_var = tk.StringVar()
        self.warmup_var = tk.StringVar(value="2")
        self.duration_var = tk.StringVar(value="10")
        self.preview_duration_var = tk.StringVar(value="5")
        self.test_style_var = tk.StringVar(value="benchmark")
        self.current_test_var = tk.StringVar(value="当前未进行测试")
        self.preview_status_var = tk.StringVar(value="基准测试不打开画面；原生纯净预览会打开独立 FFplay 窗口。")
        self.ffplay_status_var = tk.StringVar(value="")
        self.session_var = tk.StringVar(value=f"会话：{self.session_id[:8]}")
        self.custom_format_var = tk.StringVar(value="MJPG")
        self.custom_width_var = tk.StringVar(value="1920")
        self.custom_height_var = tk.StringVar(value="1080")
        self.custom_fps_var = tk.StringVar(value="30")
        self.status_var = tk.StringVar(value="等待设备枚举")
        self._build_ui()
        self.update_ffplay_status()
        self.root.after(80, self._drain_events)
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        if self.ffmpeg_var.get():
            self.root.after(250, self.refresh_devices)
        else:
            self.status_var.set("未找到 FFmpeg：请把 ffmpeg 放在脚本旁或手动选择")

    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=10); outer.pack(fill="both", expand=True)
        engine = ttk.LabelFrame(outer, text="采集引擎", padding=8); engine.pack(fill="x")
        ttk.Label(engine, text="FFmpeg：").pack(side="left")
        ttk.Entry(engine, textvariable=self.ffmpeg_var).pack(side="left", fill="x", expand=True, padx=5)
        ttk.Button(engine, text="选择…", command=self.choose_ffmpeg).pack(side="left", padx=3)
        ttk.Button(engine, text="验证", command=self.validate_ffmpeg).pack(side="left", padx=3)
        ttk.Label(engine, textvariable=self.ffplay_status_var, foreground="#555").pack(side="left", padx=(10, 0))

        device_bar = ttk.LabelFrame(outer, text="设备与能力（此阶段不运行长时间测试）", padding=8)
        device_bar.pack(fill="x", pady=(8, 0))
        ttk.Label(device_bar, text="视频设备：").pack(side="left")
        self.device_combo = ttk.Combobox(device_bar, textvariable=self.device_var, state="readonly", width=58)
        self.device_combo.pack(side="left", fill="x", expand=True, padx=5)
        self.device_combo.bind("<<ComboboxSelected>>", self.on_device_selected)
        ttk.Button(device_bar, text="刷新设备", command=self.refresh_devices).pack(side="left", padx=3)
        ttk.Button(device_bar, text="读取能力", command=self.load_capabilities).pack(side="left", padx=3)
        ttk.Button(device_bar, text="新建/切换设备会话", command=self.new_session).pack(side="left", padx=3)
        ttk.Label(device_bar, textvariable=self.session_var, foreground="#555").pack(side="left", padx=(8, 0))

        self.notebook = ttk.Notebook(outer); self.notebook.pack(fill="both", expand=True, pady=8)
        self._build_capability_tab(); self._build_plan_tab(); self._build_preview_tab(); self._build_result_tab(); self._build_raw_tab()
        bottom = ttk.Frame(outer); bottom.pack(fill="x")
        ttk.Label(bottom, textvariable=self.status_var).pack(side="left")
        self.progress = ttk.Progressbar(bottom, maximum=100, length=280); self.progress.pack(side="right")

    def _tree(self, parent, columns: list[tuple[str, str, int]], selectmode="extended"):
        frame = ttk.Frame(parent); frame.pack(fill="both", expand=True)
        tree = ttk.Treeview(frame, columns=[c[0] for c in columns], show="headings", selectmode=selectmode)
        for key, heading, width in columns:
            tree.heading(key, text=heading); tree.column(key, width=width, anchor="center" if key not in ("evidence", "reason") else "w")
        y = ttk.Scrollbar(frame, orient="vertical", command=tree.yview); x = ttk.Scrollbar(frame, orient="horizontal", command=tree.xview)
        tree.configure(yscrollcommand=y.set, xscrollcommand=x.set)
        tree.grid(row=0, column=0, sticky="nsew"); y.grid(row=0, column=1, sticky="ns"); x.grid(row=1, column=0, sticky="ew")
        frame.rowconfigure(0, weight=1); frame.columnconfigure(0, weight=1)
        return tree

    def _build_capability_tab(self):
        tab = ttk.Frame(self.notebook, padding=8); self.notebook.add(tab, text="1. 声明能力")
        toolbar = ttk.Frame(tab); toolbar.pack(fill="x", pady=(0, 6))
        ttk.Button(toolbar, text="加入选中模式", command=self.add_selected_modes).pack(side="left")
        ttk.Button(toolbar, text="加入全部模式", command=self.add_all_modes).pack(side="left", padx=5)
        ttk.Label(toolbar, text="双击行也可加入；读取能力本身不会执行长时间测试。", foreground="#555").pack(side="left", padx=10)
        self.mode_tree = self._tree(tab, [
            ("fmt", "格式/FourCC", 105), ("size", "分辨率", 115), ("fps", "FPS", 90),
            ("kind", "FPS 类型", 90), ("source", "证据来源", 190), ("evidence", "原始证据", 430),
        ])
        self.mode_tree.bind("<Double-1>", lambda _e: self.add_selected_modes())

    def _build_plan_tab(self):
        tab = ttk.Frame(self.notebook, padding=8); self.notebook.add(tab, text="2. 测试计划")
        settings = ttk.Frame(tab); settings.pack(fill="x", pady=(0, 6))
        for label, variable, width in [("恢复预热(s)", self.warmup_var, 7), ("严格测量(s)", self.duration_var, 8),
                                       ("每模式预览(s)", self.preview_duration_var, 7)]:
            ttk.Label(settings, text=label + "：").pack(side="left"); ttk.Entry(settings, textvariable=variable, width=width).pack(side="left", padx=(0, 8))
        ttk.Button(settings, text="删除选中", command=self.remove_plan).pack(side="left", padx=3)
        ttk.Button(settings, text="清空", command=self.clear_plan).pack(side="left", padx=3)

        interaction = ttk.LabelFrame(tab, text="测试交互方式", padding=6); interaction.pack(fill="x", pady=(0, 7))
        self.benchmark_radio = ttk.Radiobutton(
            interaction, text="基准测试（默认、无画面、最低开销）",
            variable=self.test_style_var, value="benchmark",
        )
        self.interactive_radio = ttk.Radiobutton(
            interaction, text="先纯净预览，再基准测量（原始流、需 FFplay）",
            variable=self.test_style_var, value="interactive",
        )
        self.benchmark_radio.pack(side="left", padx=(2, 14))
        self.interactive_radio.pack(side="left")
        self.run_button = ttk.Button(interaction, text="执行测试计划", command=self.run_plan); self.run_button.pack(side="right", padx=3)
        self.stop_button = ttk.Button(interaction, text="停止", command=self.stop, state="disabled"); self.stop_button.pack(side="right", padx=3)
        self.plan_tree = self._tree(tab, [("order", "序号", 60), ("fmt", "格式", 100), ("size", "分辨率", 120),
                                                ("fps", "FPS", 90), ("source", "来源", 240)])
        custom = ttk.LabelFrame(tab, text="手动模式（用于驱动未能枚举但需要验证的组合）", padding=6); custom.pack(fill="x", pady=(7, 0))
        for label, variable, width in [("格式", self.custom_format_var, 9), ("宽", self.custom_width_var, 8),
                                       ("高", self.custom_height_var, 8), ("FPS", self.custom_fps_var, 8)]:
            ttk.Label(custom, text=label + "：").pack(side="left"); ttk.Entry(custom, textvariable=variable, width=width).pack(side="left", padx=(0, 7))
        ttk.Button(custom, text="加入手动模式", command=self.add_custom_mode).pack(side="left")

    def _build_preview_tab(self):
        self.preview_tab = ttk.Frame(self.notebook, padding=8)
        self.notebook.add(self.preview_tab, text="3. 实时画面")
        note = (
            "每个模式先在独立 FFplay 窗口显示摄像头原始捕获流：视频区域不叠字、不裁剪、不转码；"
            "预览关闭并释放设备后，脚本再以相同模式重新打开摄像头、恢复预热并严格测量，避免显示负载制造假丢帧。"
        )
        ttk.Label(self.preview_tab, text=note, foreground="#555", wraplength=1100).pack(fill="x", pady=(0, 6))
        self.current_test_label = tk.Label(
            self.preview_tab, textvariable=self.current_test_var, background="#174a7e", foreground="white",
            font=("TkDefaultFont", 14, "bold"), padx=10, pady=8, anchor="center",
        )
        self.current_test_label.pack(fill="x", pady=(0, 6))
        self.preview_label = tk.Label(
            self.preview_tab, background="#111111", foreground="#dddddd",
            text="选择“原生纯净预览”并执行测试后，\n真实摄像头画面将在独立 FFplay 窗口中显示。",
            font=("TkDefaultFont", 14), anchor="center",
        )
        self.preview_label.pack(fill="both", expand=True)
        ttk.Label(self.preview_tab, textvariable=self.preview_status_var, anchor="center").pack(fill="x", pady=(6, 0))

    def _build_result_tab(self):
        tab = ttk.Frame(self.notebook, padding=8); self.result_tab = tab; self.notebook.add(tab, text="4. 实测结果")
        toolbar = ttk.Frame(tab); toolbar.pack(fill="x", pady=(0, 6))
        ttk.Button(toolbar, text="导出 JSON 报告", command=self.export_report).pack(side="left")
        self.result_tree = self._tree(tab, [
            ("status", "结论", 80), ("mode", "请求模式", 220), ("actual", "实际帧", 140),
            ("fps", "实测 FPS", 90), ("frames", "帧数", 70), ("p95", "间隔 P95(ms)", 110),
            ("jitter", "抖动(ms)", 100), ("drops", "疑似丢帧", 90), ("reason", "失败原因", 360),
        ], selectmode="browse")
        self.result_tree.bind("<<TreeviewSelect>>", self.show_result_detail)

    def _build_raw_tab(self):
        tab = ttk.Frame(self.notebook, padding=8); self.raw_tab = tab; self.notebook.add(tab, text="原始证据与日志")
        self.raw_text = tk.Text(tab, wrap="none", font=("TkFixedFont", 10))
        y = ttk.Scrollbar(tab, orient="vertical", command=self.raw_text.yview); x = ttk.Scrollbar(tab, orient="horizontal", command=self.raw_text.xview)
        self.raw_text.configure(yscrollcommand=y.set, xscrollcommand=x.set)
        self.raw_text.grid(row=0, column=0, sticky="nsew"); y.grid(row=0, column=1, sticky="ns"); x.grid(row=1, column=0, sticky="ew")
        tab.rowconfigure(0, weight=1); tab.columnconfigure(0, weight=1)

    def emit(self, kind: str, payload: Any = None):
        self.events.put((kind, payload))

    def reset_preview(self, text: str):
        self.preview_label.configure(text=text)
        self.preview_status_var.set(text)

    def update_ffplay_status(self):
        ffplay = locate_ffplay(self.ffmpeg_var.get().strip())
        self.ffplay_status_var.set("FFplay：已找到" if ffplay else "FFplay：未找到（仅影响原生预览）")

    def start_worker(self, func, status: str) -> bool:
        if self.worker and self.worker.is_alive():
            messagebox.showinfo("任务进行中", "请先等待当前任务结束或点击停止。")
            return False
        self.stop_event.clear(); self.status_var.set(status); self.progress["value"] = 0
        def wrapper():
            try: func()
            except Exception as exc: self.emit("error", f"{type(exc).__name__}: {exc}")
            finally: self.emit("idle")
        self.worker = threading.Thread(target=wrapper, daemon=True); self.worker.start(); return True

    def choose_ffmpeg(self):
        path = filedialog.askopenfilename(title="选择 FFmpeg", filetypes=[("FFmpeg", "ffmpeg.exe" if os.name == "nt" else "ffmpeg"), ("所有文件", "*.*")])
        if path: self.ffmpeg_var.set(path); self.validate_ffmpeg()

    def validate_ffmpeg(self):
        path = locate_ffmpeg(self.ffmpeg_var.get().strip())
        if not path:
            messagebox.showerror("FFmpeg 不可用", "无法执行所选 FFmpeg。请把官方/可信构建放在脚本同目录。")
            return
        self.ffmpeg_var.set(path); self.status_var.set(ffmpeg_version(path)); self.update_ffplay_status(); self.refresh_devices()

    def selected_device(self) -> Optional[Device]:
        index = self.device_combo.current(); return self.devices[index] if 0 <= index < len(self.devices) else None

    def has_session_data(self) -> bool:
        return bool(self.capability_raw or self.modes or self.plan or self.results)

    def _select_device_without_event(self, device: Optional[Device]):
        self._device_change_guard = True
        try:
            if device:
                index = next((i for i, item in enumerate(self.devices) if item.id == device.id), -1)
                if index >= 0:
                    self.device_combo.current(index)
                else:
                    self.device_combo.set("")
            else:
                self.device_combo.set("")
        finally:
            self._device_change_guard = False

    def reset_session_state(self, device: Optional[Device] = None):
        self.session_id = uuid.uuid4().hex
        self.session_started_at = now_iso()
        self.session_device_id = device.id if device else None
        self.session_device_snapshot = device
        self.session_var.set(f"会话：{self.session_id[:8]}")
        self.modes.clear(); self.mode_by_item.clear()
        self.plan.clear(); self.plan_by_item.clear()
        self.results.clear()
        self.capability_raw = ""; self.enumeration_raw = ""
        self.mode_tree.delete(*self.mode_tree.get_children())
        self.plan_tree.delete(*self.plan_tree.get_children())
        self.result_tree.delete(*self.result_tree.get_children())
        self.raw_text.delete("1.0", "end")
        self.progress["value"] = 0
        self.current_test_var.set("当前未进行测试")
        self.preview_status_var.set("新会话已就绪；尚未运行预览或严格测量。")
        self._select_device_without_event(device)
        self.notebook.select(0)
        if device:
            self.status_var.set(f"新会话已建立：{device.label}；请点击“读取能力”")
        else:
            self.status_var.set("新会话已建立；请选择要测试的视频设备")

    def new_session(self):
        if self.worker and self.worker.is_alive():
            messagebox.showinfo("任务进行中", "请先等待当前任务结束或点击停止，再新建会话。")
            return
        if self.has_session_data() and not messagebox.askokcancel(
            "新建测试会话",
            "这会清空当前能力表、测试计划、结果和日志。尚未导出的结果不会自动保存。\n\n是否继续？",
        ):
            return
        self.reset_session_state(None)
        self.refresh_devices()

    def on_device_selected(self, _event=None):
        if self._device_change_guard:
            return
        device = self.selected_device()
        if not device:
            return
        if self.worker and self.worker.is_alive():
            previous = next((item for item in self.devices if item.id == self.session_device_id), None)
            self._select_device_without_event(previous)
            messagebox.showinfo("任务进行中", "任务运行期间不能切换设备。")
            return
        if self.session_device_id and device.id != self.session_device_id and self.has_session_data():
            if not messagebox.askokcancel(
                "切换设备并新建会话",
                f"当前会话属于另一台设备。切换到“{device.label}”将清空能力、计划、结果和日志。\n\n是否继续？",
            ):
                previous = next((item for item in self.devices if item.id == self.session_device_id), None)
                self._select_device_without_event(previous)
                return
            self.reset_session_state(device)
            self.refresh_devices()
            return
        self.session_device_id = device.id
        self.session_device_snapshot = device
        self.status_var.set(f"当前会话设备：{device.label}；请点击“读取能力”")

    def require_ffmpeg(self) -> Optional[str]:
        path = locate_ffmpeg(self.ffmpeg_var.get().strip())
        if not path: messagebox.showerror("缺少 FFmpeg", "请把 ffmpeg 放在脚本旁，或点击“选择”。")
        return path

    def refresh_devices(self):
        ffmpeg = self.require_ffmpeg()
        if not ffmpeg: return
        def work():
            devices, raw = enumerate_devices(ffmpeg); self.emit("devices", (devices, raw))
        self.start_worker(work, "正在枚举视频设备…")

    def load_capabilities(self):
        ffmpeg, device = self.require_ffmpeg(), self.selected_device()
        if not ffmpeg or not device:
            messagebox.showwarning("未选择设备", "请先选择视频设备。")
            return
        self.session_device_id = device.id; self.session_device_snapshot = device
        def work():
            modes, raw = read_capabilities(ffmpeg, device); self.emit("capabilities", (modes, raw))
        self.start_worker(work, "正在读取驱动声明能力；不会执行长时间测试…")

    def render_modes(self):
        self.mode_tree.delete(*self.mode_tree.get_children()); self.mode_by_item.clear()
        for mode in self.modes:
            item = self.mode_tree.insert("", "end", values=(mode.format_label, f"{mode.width}×{mode.height}", f"{mode.fps:g}",
                                                               mode.fps_kind, mode.source, mode.evidence))
            self.mode_by_item[item] = mode

    def add_modes(self, modes: list[Mode]):
        existing = {mode.key() for mode in self.plan}
        for mode in modes:
            if mode.key() not in existing:
                self.plan.append(mode); existing.add(mode.key())
        self.render_plan(); self.notebook.select(1)

    def add_selected_modes(self):
        selected = [self.mode_by_item[item] for item in self.mode_tree.selection() if item in self.mode_by_item]
        if not selected: messagebox.showinfo("未选择模式", "请在能力表中选择一行或多行。")
        else: self.add_modes(selected)

    def add_all_modes(self):
        if self.modes and messagebox.askokcancel("加入全部模式", f"将加入 {len(self.modes)} 个模式。完整测试可能耗时很长，是否继续？"):
            self.add_modes(self.modes)

    def add_custom_mode(self):
        try:
            fmt = self.custom_format_var.get().strip(); width = int(self.custom_width_var.get()); height = int(self.custom_height_var.get()); fps = float(self.custom_fps_var.get())
            if not fmt or min(width, height, fps) <= 0: raise ValueError
        except ValueError:
            messagebox.showerror("参数错误", "请输入有效格式、宽、高和 FPS。")
            return
        device = self.selected_device(); option_kind = "vcodec" if fmt.upper() in {"MJPG", "MJPEG", "H264", "HEVC", "H265"} and device and device.backend == "dshow" else "pixel_format"
        input_fmt = {"MJPG": "mjpeg", "MJPEG": "mjpeg"}.get(fmt.upper(), fmt.lower())
        self.add_modes([Mode(fmt.upper(), input_fmt, width, height, fps, "manual", option_kind, "用户手动")])

    def render_plan(self):
        self.plan_tree.delete(*self.plan_tree.get_children()); self.plan_by_item.clear()
        for index, mode in enumerate(self.plan, 1):
            item = self.plan_tree.insert("", "end", values=(index, mode.format_label, f"{mode.width}×{mode.height}", f"{mode.fps:g}", mode.source))
            self.plan_by_item[item] = mode

    def remove_plan(self):
        remove = {self.plan_by_item[item].key() for item in self.plan_tree.selection() if item in self.plan_by_item}
        self.plan = [mode for mode in self.plan if mode.key() not in remove]; self.render_plan()

    def clear_plan(self):
        self.plan.clear(); self.render_plan()

    def run_plan(self):
        ffmpeg, device = self.require_ffmpeg(), self.selected_device()
        if not ffmpeg or not device: return
        self.session_device_id = device.id; self.session_device_snapshot = device
        if not self.plan:
            messagebox.showinfo("测试计划为空", "请先从能力表选择模式加入测试计划。")
            return
        try:
            warmup, duration = float(self.warmup_var.get()), float(self.duration_var.get())
            if not 0 <= warmup <= 60 or not 1 <= duration <= 3600: raise ValueError
        except ValueError:
            messagebox.showerror("时间参数错误", "预热范围 0–60 秒；每模式测试时长 1–3600 秒。")
            return
        preview_enabled = self.test_style_var.get() == "interactive"
        try:
            preview_duration = float(self.preview_duration_var.get()) if preview_enabled else 0.0
            if preview_enabled and not 1 <= preview_duration <= 120: raise ValueError
        except ValueError:
            messagebox.showerror("预览时间错误", "原生纯净预览时间范围为 1–120 秒。")
            return
        ffplay = locate_ffplay(ffmpeg) if preview_enabled else None
        if preview_enabled and not ffplay:
            messagebox.showerror(
                "缺少 FFplay",
                "原生纯净预览需要 ffplay。请把与 ffmpeg 同一工具包中的 ffplay.exe（Windows）或 ffplay 放到脚本或 ffmpeg 旁边。\n\n基准测试不需要 FFplay。",
            )
            self.update_ffplay_status()
            return
        estimate = len(self.plan) * (preview_duration + warmup + duration)
        preview_note = (
            f"两阶段交互：先显示 {preview_duration:g} 秒原始流纯净画面；关闭画面并恢复预热后，再进行不受显示负载影响的严格测量。"
            if preview_enabled else "基准测试：不显示画面，保持最低测量开销。"
        )
        if not messagebox.askokcancel(
            "执行严格测试",
            f"共 {len(self.plan)} 个模式，最低预计 {estimate/60:.1f} 分钟。\n{preview_note}\n设备将被独占，是否开始？",
        ):
            return
        self.results = []; self.result_tree.delete(*self.result_tree.get_children())
        self.run_button.configure(state="disabled"); self.stop_button.configure(state="normal")
        self.benchmark_radio.configure(state="disabled"); self.interactive_radio.configure(state="disabled")
        if preview_enabled:
            self.reset_preview("即将打开 FFplay 原生纯净画面窗口…")
            self.current_test_var.set("原生纯净预览即将开始…")
            self.notebook.select(self.preview_tab)
        def work():
            for index, mode in enumerate(list(self.plan), 1):
                if self.stop_event.is_set(): break
                self.emit("current_mode", f"当前测试 {index}/{len(self.plan)} · {mode.label()}")
                preview_record = None
                if preview_enabled:
                    self.emit("status", f"纯净画面观察 {index}/{len(self.plan)}：{mode.label()}")
                    self.emit("preview_status", f"阶段 1/2：正在显示 {mode.label()} 的原始捕获流；尚未计入严格测量。")
                    try:
                        preview_record = run_native_preview(
                            ffmpeg, ffplay, device, mode, preview_duration, self.stop_event,
                            lambda p: self.emit("process", p),
                            lambda value: self.emit("plan_progress", ((index - 1) + (value / 100) * (preview_duration / max(preview_duration + warmup + duration, 0.1))) / len(self.plan) * 100),
                        )
                    except Exception as exc:
                        preview_record = {"status": "ERROR", "error": f"{type(exc).__name__}: {exc}",
                                          "overlaps_strict_measurement": False}
                    if self.stop_event.is_set(): break
                    self.emit("preview_status", "阶段 2/2：纯净画面已关闭；正在恢复预热并进行无预览严格测量。")
                self.emit("status", f"严格测量 {index}/{len(self.plan)}：{mode.label()}")
                strict_span = warmup + duration
                completed_before_strict = preview_duration
                total_span = max(preview_duration + strict_span, 0.1)
                def item_progress(value):
                    fraction = (completed_before_strict + value / 100 * strict_span) / total_span
                    self.emit("plan_progress", ((index - 1) + fraction) / len(self.plan) * 100)
                result = run_strict_test(ffmpeg, device, mode, warmup, duration, self.stop_event,
                                         lambda p: self.emit("process", p), item_progress)
                result["device"] = asdict(device); result["tested_at"] = now_iso()
                result["interaction_mode"] = "preview_then_benchmark" if preview_enabled else "benchmark_no_preview"
                result["preview_observation"] = preview_record
                self.emit("test_result", result)
        self.start_worker(work, "正在执行严格测试计划…")

    def stop(self):
        self.stop_event.set(); self.status_var.set("正在停止当前 FFmpeg 进程…")
        process = self.current_process
        if process and process.poll() is None:
            try: process.terminate()
            except OSError: pass

    def append_result(self, result: dict[str, Any]):
        self.results.append(result); requested = result["requested"]; actual = result["actual"]; measurement = result["measurement"]
        values = (result["status"], f"{requested['format_label']} {requested['width']}×{requested['height']}@{requested['fps']:g}",
                  f"{actual.get('width')}×{actual.get('height')}" if actual.get("width") else "—",
                  measurement.get("measured_fps_from_pts"), measurement.get("showinfo_frames"),
                  measurement.get("frame_interval_p95_ms"), measurement.get("frame_interval_jitter_std_ms"),
                  measurement.get("estimated_dropped_frames_from_pts"), "; ".join(result.get("failure_reasons", [])))
        item = self.result_tree.insert("", "end", values=values); self.result_tree.set(item, "status", result["status"])
        self.result_tree.item(item, tags=(f"result_{len(self.results)-1}",))

    def show_result_detail(self, _event=None):
        selection = self.result_tree.selection()
        if not selection: return
        tags = self.result_tree.item(selection[0], "tags")
        if not tags: return
        index = int(tags[0].split("_")[-1]); self.raw_text.delete("1.0", "end"); self.raw_text.insert("1.0", json.dumps(self.results[index], ensure_ascii=False, indent=2)); self.notebook.select(self.raw_tab)

    def report(self) -> dict[str, Any]:
        ffmpeg = self.ffmpeg_var.get().strip(); ffplay = locate_ffplay(ffmpeg); device = self.session_device_snapshot or self.selected_device()
        return {"schema_version": 5, "application": {"name": APP_NAME, "version": APP_VERSION}, "generated_at": now_iso(),
                "session": {"id": self.session_id, "started_at": self.session_started_at,
                            "device_id": self.session_device_id},
                "environment": {"os": platform.platform(), "python": sys.version, "ffmpeg": ffmpeg_version(ffmpeg) if ffmpeg else None,
                                "ffmpeg_path": ffmpeg, "ffplay_path": ffplay},
                "device": asdict(device) if device else None, "declared_modes": [asdict(mode) for mode in self.modes],
                "test_plan": [asdict(mode) for mode in self.plan], "selected_interaction_mode": self.test_style_var.get(), "results": self.results,
                "capability_raw": self.capability_raw, "enumeration_raw": self.enumeration_raw,
                "method_notes": ["PASS 要求 FFmpeg 正常退出、分辨率严格匹配、PTS 实测 FPS 在 max(0.5,5%) 容差内，且帧数达到请求值的 75%。",
                                 "MJPEG/H.264 解码后的像素格式不等同于输入 FourCC，因此只作为实际解码证据记录。",
                                 "PTS 间隔由 FFmpeg showinfo 得到；疑似丢帧为基于请求周期的估算。",
                                 "两阶段交互先以相同请求模式独立打开摄像头，将捕获流不经视频转码直接送往 FFplay；画面中不叠加文字。",
                                 "预览结束并释放设备后，严格测量重新以相同模式打开摄像头，经过恢复预热再采样；预览负载不与 PASS/FAIL 测量重叠。"]}

    def export_report(self):
        if not self.results:
            messagebox.showinfo("暂无结果", "请先执行测试计划。")
            return
        device = self.session_device_snapshot or self.selected_device()
        device_name = re.sub(r"[^0-9A-Za-z._-]+", "_", device.label if device else "video_device").strip("_")[:40]
        initial = f"{device_name}_{self.session_id[:8]}_{datetime.now():%Y%m%d_%H%M%S}.json"
        path = filedialog.asksaveasfilename(defaultextension=".json", initialfile=initial, filetypes=[("JSON", "*.json")])
        if path:
            try: Path(path).write_text(json.dumps(self.report(), ensure_ascii=False, indent=2), encoding="utf-8"); self.status_var.set(f"报告已保存：{path}")
            except OSError as exc: messagebox.showerror("保存失败", str(exc))

    def _drain_events(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "devices":
                    previous_id = self.session_device_id
                    self.devices, self.enumeration_raw = payload; self.device_combo["values"] = [d.label for d in self.devices]
                    previous = next((item for item in self.devices if item.id == previous_id), None)
                    if previous:
                        self._select_device_without_event(previous)
                        self.session_device_snapshot = previous
                        self.status_var.set(f"发现 {len(self.devices)} 个设备；当前会话设备仍为 {previous.label}")
                    elif self.devices and not self.has_session_data():
                        first = self.devices[0]
                        self._select_device_without_event(first)
                        self.session_device_id = first.id; self.session_device_snapshot = first
                        self.status_var.set(f"发现 {len(self.devices)} 个设备；当前选择 {first.label}，可切换后读取能力")
                    elif self.devices:
                        self._select_device_without_event(None)
                        self.status_var.set("当前会话设备已不在列表中；请导出结果后新建/切换设备会话")
                    else:
                        self._select_device_without_event(None); self.status_var.set("没有枚举到视频设备")
                    self.raw_text.delete("1.0", "end"); self.raw_text.insert("1.0", self.enumeration_raw)
                elif kind == "capabilities":
                    self.modes, self.capability_raw = payload; self.render_modes(); self.status_var.set(f"读取到 {len(self.modes)} 个格式/分辨率/FPS 组合")
                    self.raw_text.delete("1.0", "end"); self.raw_text.insert("1.0", self.capability_raw)
                elif kind == "test_result": self.append_result(payload)
                elif kind == "current_mode":
                    self.current_test_var.set(str(payload))
                    if self.test_style_var.get() == "interactive": self.reset_preview("准备显示当前模式的无覆盖真实画面…")
                elif kind == "preview_status": self.reset_preview(str(payload))
                elif kind == "plan_progress": self.progress["value"] = payload
                elif kind == "status": self.status_var.set(str(payload))
                elif kind == "process": self.current_process = payload
                elif kind == "error": self.status_var.set("操作失败"); messagebox.showerror("操作失败", str(payload))
                elif kind == "idle":
                    self.current_process = None; self.run_button.configure(state="normal"); self.stop_button.configure(state="disabled")
                    self.benchmark_radio.configure(state="normal"); self.interactive_radio.configure(state="normal")
                    if self.results:
                        self.status_var.set(f"测试结束：PASS {sum(r['status']=='PASS' for r in self.results)} / {len(self.results)}")
                        self.current_test_var.set("测试计划已结束")
                        self.preview_status_var.set("测试结束；结果中已记录原生预览窗口及播放器状态。")
                        self.notebook.select(self.result_tab)
        except queue.Empty:
            pass
        self.root.after(80, self._drain_events)

    def _close(self):
        self.stop(); self.root.destroy()


def main() -> int:
    parser = argparse.ArgumentParser(description=APP_NAME)
    parser.add_argument("--ffmpeg", default="", help="FFmpeg 可执行文件路径")
    args = parser.parse_args()
    root = tk.Tk(); StrictTesterApp(root, args.ffmpeg); root.mainloop(); return 0


if __name__ == "__main__":
    raise SystemExit(main())
