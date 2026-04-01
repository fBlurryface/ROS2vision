import argparse
import os
import signal
import subprocess
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix


def normalize_namespace(ns: str) -> str:
    if not ns or ns == "/":
        return ""
    return "/" + ns.lstrip("/")


def device_ready(device: str) -> bool:
    if not os.path.exists(device):
        return False

    try:
        result = subprocess.run(
            ["v4l2-ctl", "--list-formats-ext", f"--device={device}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=2.0,
            check=False,
        )
        return result.returncode == 0
    except Exception:
        return False


def build_usb_cam_command(args) -> list[str]:
    usb_cam_prefix = Path(get_package_prefix("usb_cam"))
    usb_cam_exe = usb_cam_prefix / "lib" / "usb_cam" / "usb_cam_node_exe"

    cmd = [
        str(usb_cam_exe),
        "--ros-args",
        "-r", "__node:=camera_source",
    ]

    ns = normalize_namespace(args.namespace)
    if ns:
        cmd += ["-r", f"__ns:={ns}"]

    cmd += [
        "--params-file", args.params_file,
        "-p", f"video_device:={args.video_device}",
        "-p", f"camera_name:={args.camera_name}",
        "-p", f"frame_id:={args.frame_id}",
    ]

    return cmd


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--params-file", required=True)
    parser.add_argument("--video-device", required=True)
    parser.add_argument("--camera-name", default="front_camera")
    parser.add_argument("--frame-id", default="camera_optical_frame")
    parser.add_argument("--namespace", default="camera")
    parser.add_argument("--retry-delay", type=float, default=2.5)

    args, _unknown = parser.parse_known_args()

    child = None

    try:
        while True:
            while not device_ready(args.video_device):
                print(f"[camera_runner] waiting for device: {args.video_device}", flush=True)
                time.sleep(args.retry_delay)

            cmd = build_usb_cam_command(args)
            print(f"[camera_runner] starting usb_cam: {' '.join(cmd)}", flush=True)

            child = subprocess.Popen(cmd)
            return_code = child.wait()

            print(f"[camera_runner] usb_cam exited with code {return_code}", flush=True)
            child = None
            time.sleep(args.retry_delay)

    except KeyboardInterrupt:
        print("[camera_runner] interrupted, shutting down", flush=True)
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                child.kill()


if __name__ == "__main__":
    main()
