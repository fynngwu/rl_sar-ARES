#!/usr/bin/env -S uv run python
"""
Camera Math Solver → Auto Pilot pipeline.

1. Capture frame from USB camera
2. LLM solves math problem → returns answer number
3. Generate auto_pilot YAML config from answer
4. Launch auto_pilot.py

Usage:
    uv run python tools/camera_math_solver.py
    uv run python tools/camera_math_solver.py --camera 1
    uv run python tools/camera_math_solver.py --no-auto
"""

import argparse
import base64
import os
import re
import subprocess
import sys
import tempfile
import time

import cv2
import requests
import yaml


API_URL = "https://opencode.ai/zen/go/v1/chat/completions"
API_KEY = "sk-QeaUeQ0atfgjYiAWcuIjamkbI8PqytF78Fuou0i9lwKuMaaeqWgTAjj1S7SYMT3V"

MATH_PROMPT = (
    "请仔细观察这张图片中的数学题，计算出最终答案。"
    "请只回复一个数字，不要有任何其他文字、解释或符号。"
    "例如: 42"
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
AUTO_PILOT_YAML = os.path.join(SCRIPT_DIR, "auto_pilot_config.yaml")


def capture_frame(camera_id: int = 0) -> str:
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {camera_id}")
        sys.exit(1)

    time.sleep(0.5)
    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("[ERROR] Failed to capture frame")
        sys.exit(1)

    h, w = frame.shape[:2]
    print(f"[CAM] Captured {w}x{h} from camera {camera_id}")

    _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return base64.b64encode(buf.tobytes()).decode("utf-8"), frame


def ask_vision(base64_img: str, prompt: str) -> str:
    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json",
    }
    payload = {
        "model": "gpt-4o",
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{base64_img}",
                            "detail": "high",
                        },
                    },
                ],
            }
        ],
        "max_tokens": 256,
        "temperature": 0.0,
    }

    resp = requests.post(API_URL, headers=headers, json=payload, timeout=60)
    resp.raise_for_status()
    data = resp.json()
    return data["choices"][0]["message"]["content"].strip()


def extract_number(text: str) -> float:
    """Extract the first number from LLM response."""
    match = re.search(r"-?\d+\.?\d*", text)
    if not match:
        print(f"[ERROR] No number found in LLM response: {text}")
        sys.exit(1)
    return float(match.group())


def generate_config(answer: float, output_path: str):
    """Generate auto_pilot YAML config from math answer."""
    vx = max(-2.0, min(2.0, answer * 0.1))

    config = {
        "steps": [
            {"vx": 0.0, "vy": 0.0, "wz": 0.0, "duration": 1.0, "smooth": False},
            {"vx": round(vx, 3), "vy": 0.0, "wz": 0.0, "duration": 3.0, "smooth": True},
            {"vx": 0.0, "vy": 0.0, "wz": 0.0, "duration": 2.0, "smooth": True},
        ],
    }

    with open(output_path, "w") as f:
        yaml.dump(config, f, default_flow_style=False, allow_unicode=True)

    print(f"[CONFIG] Answer={answer} → vx={vx:.3f} m/s")
    print(f"[CONFIG] Written to {output_path}")


def launch_auto_pilot(config_path: str):
    print(f"[LAUNCH] Starting auto_pilot with {config_path} ...")
    script = os.path.join(SCRIPT_DIR, "auto_pilot.py")
    subprocess.run([sys.executable, script, "--config", config_path])


def main():
    parser = argparse.ArgumentParser(description="Camera Math Solver → Auto Pilot")
    parser.add_argument("--camera", "-c", type=int, default=0, help="Camera device ID")
    parser.add_argument("--save", "-s", type=str, default=None, help="Save captured image")
    parser.add_argument("--no-auto", action="store_true", help="Only solve, don't launch auto_pilot")
    args = parser.parse_args()

    print("[1/4] Capturing image...")
    b64, frame = capture_frame(args.camera)

    if args.save:
        cv2.imwrite(args.save, frame)
        print(f"[CAM] Saved to {args.save}")

    print("[2/4] Solving math problem...")
    raw = ask_vision(b64, MATH_PROMPT)
    print(f"[LLM] Raw response: {raw}")

    answer = extract_number(raw)
    print(f"[LLM] Final answer: {answer}")

    print("[3/4] Generating config...")
    generate_config(answer, AUTO_PILOT_YAML)

    if not args.no_auto:
        print("[4/4] Launching auto_pilot...")
        launch_auto_pilot(AUTO_PILOT_YAML)
    else:
        print("[4/4] Skipped (--no-auto)")


if __name__ == "__main__":
    main()
