"""Create the robot records the fake-hardware rig records with.

    HOME=/tmp/fakehome python tests/fake_hw/setup_fake_robots.py

Writes, under $HOME/.cache/huggingface/lerobot/:

- placeholder calibration files for a "left" and a "right" DK1 arm pair
  (the DK1 needs no calibration, see lelab/robot_profile.py);
- robot records "left" (paired with "right" as its right arm, and carrying
  four fake cameras) and "right".

Always run it with a throwaway HOME so it cannot touch your real robots,
calibrations or datasets.
"""

import argparse
import json
import os
import pwd
from pathlib import Path

CAMERAS = [
    ("front", 640, 480),
    ("top", 640, 480),
    ("left_wrist", 480, 360),
    ("right_wrist", 480, 360),
]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--home",
        type=Path,
        default=Path(os.environ["HOME"]),
        help="throwaway HOME to populate (default: $HOME)",
    )
    args = ap.parse_args()
    # The account's own home, which $HOME cannot disguise.
    real_home = Path(pwd.getpwuid(os.getuid()).pw_dir)
    if args.home.expanduser().resolve() == real_home.resolve():
        raise SystemExit(
            f"Refusing to write to your real home ({real_home}): this would overwrite your own "
            "robot configs. Run with HOME=/tmp/lelab-fake, or pass --home."
        )

    root = args.home / ".cache" / "huggingface" / "lerobot"
    follower_dir = root / "calibration" / "robots" / "dk1_follower"
    leader_dir = root / "calibration" / "teleoperators" / "dk1_leader"
    robots_dir = root / "robots"
    for d in (follower_dir, leader_dir, robots_dir):
        d.mkdir(parents=True, exist_ok=True)

    cameras = [
        {
            "id": f"c{i}",
            "name": name,
            "device_id": f"fake{i}",
            "type": "opencv",
            "camera_index": i,
            "width": width,
            "height": height,
            "fps": 30,
            "rotation": 0,
        }
        for i, (name, width, height) in enumerate(CAMERAS, start=1)
    ]

    for side in ("left", "right"):
        (follower_dir / f"{side}_f.json").write_text("{}\n")
        (leader_dir / f"{side}_l.json").write_text("{}\n")
        record = {
            "name": side,
            "leader_port": f"/dev/fake_{side}_leader",
            "follower_port": f"/dev/fake_{side}_follower",
            "leader_config": f"{side}_l.json",
            "follower_config": f"{side}_f.json",
            # Only the left robot carries the cameras: a bimanual recording uses
            # the left (primary) robot's camera list for the whole rig.
            "cameras": cameras if side == "left" else [],
        }
        if side == "left":
            record["right_arm"] = "right"
        (robots_dir / f"{side}.json").write_text(json.dumps(record, indent=1) + "\n")

    print(f"Wrote fake robots 'left' (+ right arm 'right', {len(cameras)} cameras) and 'right' under {root}")


if __name__ == "__main__":
    main()
