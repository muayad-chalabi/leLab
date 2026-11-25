"""Record a dataset through LeLab's HTTP API, as the Recording page does.

    python tests/fake_hw/record_via_api.py --api http://127.0.0.1:8010 --robot left \\
        --episodes 2 --episode-s 8 [--no-streaming]

Sends the same /start-recording request the GUI builds from the robot record
(right arm and cameras included), presses "End Episode" (/recording-exit-early)
after --episode-s seconds of each episode, and prints the resulting repo id.
"""

import argparse
import json
import sys
import time
import urllib.request


def call(api: str, path: str, body: dict | None = None) -> dict:
    req = urllib.request.Request(
        api + path,
        data=json.dumps(body).encode() if body is not None else None,
        headers={"Content-Type": "application/json"},
        method="POST" if body is not None else "GET",
    )
    with urllib.request.urlopen(req, timeout=30) as resp:
        return json.load(resp)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--api", default="http://127.0.0.1:8010")
    ap.add_argument("--robot", default="left")
    ap.add_argument("--name", default="fake_api_test")
    ap.add_argument("--episodes", type=int, default=2)
    ap.add_argument("--episode-s", type=float, default=8)
    ap.add_argument("--reset-s", type=int, default=3)
    ap.add_argument("--no-streaming", action="store_true")
    args = ap.parse_args()

    robot = call(args.api, f"/robots/{args.robot}")["robot"]
    cameras = {
        c["name"]: {k: c[k] for k in ("type", "camera_index", "width", "height", "fps", "rotation") if k in c}
        for c in robot["cameras"]
    }
    body = {
        "leader_port": robot["leader_port"],
        "follower_port": robot["follower_port"],
        "leader_config": robot["leader_config"],
        "follower_config": robot["follower_config"],
        "dataset_repo_id": args.name,
        "single_task": "fake hardware fidelity test",
        "num_episodes": args.episodes,
        "episode_time_s": int(args.episode_s * 4),  # never reached: we end episodes ourselves
        "reset_time_s": args.reset_s,
        "fps": 30,
        "video": True,
        "push_to_hub": False,
        "streaming_encoding": not args.no_streaming,
        "cameras": cameras,
    }
    if robot.get("right_arm"):
        body["right_leader_port"] = robot["right_leader_port"]
        body["right_follower_port"] = robot["right_follower_port"]
    print("start:", call(args.api, "/start-recording", body).get("message"))

    ended: set[int] = set()
    while True:
        status = call(args.api, "/recording-status")
        if not status.get("recording_active"):
            break
        episode = status.get("current_episode")
        if (
            status.get("current_phase") == "recording"
            and episode not in ended
            and (status.get("phase_elapsed_seconds") or 0) >= args.episode_s
        ):
            call(args.api, "/recording-exit-early", {})
            ended.add(episode)
            print(f"ended episode {episode}")
        time.sleep(0.2)

    print(
        json.dumps(
            {k: status.get(k) for k in ("current_phase", "saved_episodes", "error", "dataset_repo_id")}
        )
    )
    sys.exit(0 if status.get("current_phase") != "error" else 1)


if __name__ == "__main__":
    main()
