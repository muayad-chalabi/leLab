"""Check a dataset recorded against the fake DK1 hardware for fidelity.

    python tests/fake_hw/check_dataset.py <dataset_root> <fake_hw.jsonl> \\
        --cameras front=1,top=2,left_wrist=3,right_wrist=4 --episodes 2 --episode-s 8

Loads the dataset through LeRobotDataset (what training sees) and compares it
with the ground truth fake_dk1 logged while recording:

- metadata: fps, robot type, feature names/order, image shapes;
- indexing: episode count and lengths, frame_index, timestamps, global index;
- actions / states: every row equals one logged leader action / follower
  observation, in order; state[k] equals action[k-1] (the fake follower tracks
  its command instantly), so rows are not shifted against each other;
- video: every decoded frame carries the camera index of its feature (no
  swapped cameras) and exactly the frame code the robot returned for that row
  (no frame shifts after encoding), plus camera latency and repeated frames.

Exits non-zero if any hard check fails.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fake_dk1 import decode_frame  # noqa: E402

JOINTS = [f"joint_{j}.pos" for j in range(1, 7)] + ["gripper.pos"]
EXPECTED_NAMES = [f"{side}_{j}" for side in ("left", "right") for j in JOINTS]

failures: list[str] = []


def check(ok: bool, message: str) -> None:
    print(("PASS " if ok else "FAIL ") + message)
    if not ok:
        failures.append(message)


def _match_rows(rows: np.ndarray, log: list[np.ndarray], tol: float = 1e-5) -> list[int | None]:
    """Index of the logged vector each row equals, searching forward in order."""
    out: list[int | None] = []
    start = 0
    for row in rows:
        found = None
        for i in range(start, len(log)):
            if np.max(np.abs(log[i] - row)) <= tol:
                found = i
                break
        out.append(found)
        if found is not None:
            start = found + 1
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("root", type=Path)
    ap.add_argument("log", type=Path)
    ap.add_argument("--cameras", required=True, help="name=camera_index,...")
    ap.add_argument("--episodes", type=int, required=True)
    ap.add_argument("--episode-s", type=float, required=True)
    args = ap.parse_args()
    cameras = {k: int(v) for k, v in (kv.split("=") for kv in args.cameras.split(","))}

    from lerobot.datasets import LeRobotDataset

    ds = LeRobotDataset(repo_id=f"local/{args.root.name}", root=args.root)
    meta = ds.meta
    fps = meta.fps

    print(f"== {args.root}\n   {meta.total_episodes} episodes, {meta.total_frames} frames @ {fps} fps")

    # ---------------------------------------------------------- metadata
    check(meta.robot_type == "bi_dk1_follower", f"robot_type is bi_dk1_follower ({meta.robot_type})")
    for key in ("action", "observation.state"):
        names = meta.features[key]["names"]
        check(list(names) == EXPECTED_NAMES, f"{key} names are left_*/right_* joints in order")
    for name in cameras:
        key = f"observation.images.{name}"
        ft = meta.features.get(key)
        check(ft is not None and ft["dtype"] == "video", f"{key} stored as video")
    check(
        {k for k in meta.features if k.startswith("observation.images.")}
        == {f"observation.images.{n}" for n in cameras},
        "no extra or missing camera features",
    )

    # ---------------------------------------------------------- indexing
    hf = ds.hf_dataset.with_format("numpy")
    ep = np.asarray(hf["episode_index"])
    fi = np.asarray(hf["frame_index"])
    ts = np.asarray(hf["timestamp"])
    idx = np.asarray(hf["index"])
    check(meta.total_episodes == args.episodes, f"{args.episodes} episodes saved ({meta.total_episodes})")
    # Episodes are ended from outside ("End Episode" once --episode-s has
    # elapsed), so allow up to 1.5 s of polling slack.
    min_len, max_len = round(args.episode_s * fps) - 2, round((args.episode_s + 1.5) * fps)
    for e in range(meta.total_episodes):
        n = int((ep == e).sum())
        check(min_len <= n <= max_len, f"episode {e}: {n} frames (expected {min_len}..{max_len})")
        check(np.array_equal(fi[ep == e], np.arange(n)), f"episode {e}: frame_index is 0..{n - 1}")
    check(np.allclose(ts, fi / fps, atol=1e-4), "timestamp == frame_index / fps")
    check(np.array_equal(idx, np.arange(len(idx))), "global index is contiguous")

    # ---------------------------------------------------------- ground-truth log
    entries = [json.loads(line) for line in args.log.read_text().splitlines()]
    t0 = next(e["v"]["t0"] for e in entries if e["kind"] == "start")
    obs_log = [e for e in entries if e["kind"] == "obs"]
    act_log = [e for e in entries if e["kind"] == "action"]
    obs_vecs = [np.array([e["v"][n] for n in EXPECTED_NAMES], np.float32) for e in obs_log]
    act_vecs = [np.array([e["v"][n] for n in EXPECTED_NAMES], np.float32) for e in act_log]

    actions = np.asarray(hf["action"], np.float32)
    states = np.asarray(hf["observation.state"], np.float32)
    act_match = _match_rows(actions, act_vecs)
    obs_match = _match_rows(states, obs_vecs)
    n_act = sum(m is not None for m in act_match)
    n_obs = sum(m is not None for m in obs_match)
    check(
        n_act == len(actions),
        f"every action row is a logged leader action, in order ({n_act}/{len(actions)})",
    )
    check(
        n_obs == len(states),
        f"every state row is a logged follower observation, in order ({n_obs}/{len(states)})",
    )

    lag_ok = lag_total = 0
    for e in range(meta.total_episodes):
        rows = np.flatnonzero(ep == e)
        for prev, cur in zip(rows[:-1], rows[1:], strict=True):
            lag_total += 1
            lag_ok += bool(np.max(np.abs(states[cur] - actions[prev])) <= 1e-5)
    check(lag_ok == lag_total, f"state[k] == action[k-1] within episodes ({lag_ok}/{lag_total})")

    # Loop timing, from the matched observation log times.
    obs_t = np.array([obs_log[m]["t"] for m in obs_match if m is not None])
    for e in range(meta.total_episodes):
        sel = np.array([ep[i] == e and obs_match[i] is not None for i in range(len(ep))])
        dt = np.diff(np.array([obs_log[obs_match[i]]["t"] for i in np.flatnonzero(sel)]))
        if len(dt):
            print(
                f"     episode {e}: loop period mean {dt.mean() * 1000:.1f} ms, "
                f"p99 {np.percentile(dt, 99) * 1000:.1f} ms, max {dt.max() * 1000:.1f} ms "
                f"(target {1000 / fps:.1f})"
            )
    check(len(obs_t) > 1, "observation log timing available")

    # ---------------------------------------------------------- video frames
    print(f"   decoding {len(ds)} x {len(cameras)} frames through LeRobotDataset...")
    wrong_cam = dict.fromkeys(cameras, 0)
    shifted = dict.fromkeys(cameras, 0)
    latency = {n: [] for n in cameras}
    repeats = dict.fromkeys(cameras, 0)
    prev_t = dict.fromkeys(cameras)
    for i in range(len(ds)):
        item = ds[i]
        m = obs_match[i]
        for name, cam_index in cameras.items():
            img = item[f"observation.images.{name}"]
            frame = (img.permute(1, 2, 0).numpy() * 255).round().astype(np.uint8)
            got_idx, got_t = decode_frame(frame)
            wrong_cam[name] += got_idx != cam_index
            if m is not None:
                logged = obs_log[m]["v"][name]
                shifted[name] += abs(got_t - logged["cam_t"]) > 0.0015 or got_idx != logged["cam_index"]
                latency[name].append((obs_log[m]["t"] - t0) - got_t)
            if i and fi[i] and prev_t[name] == got_t:
                repeats[name] += 1
            prev_t[name] = got_t
    for name in cameras:
        lat = np.array(latency[name]) * 1000
        check(
            wrong_cam[name] == 0,
            f"{name}: every frame is from camera {cameras[name]} ({wrong_cam[name]} wrong)",
        )
        check(
            shifted[name] == 0,
            f"{name}: decoded frame == frame the robot returned for that row ({shifted[name]} off)",
        )
        print(
            f"     {name}: camera latency mean {lat.mean():.1f} ms, max {lat.max():.1f} ms; "
            f"repeated frames {repeats[name]}"
        )

    print(f"\n{'ALL CHECKS PASSED' if not failures else f'{len(failures)} CHECK(S) FAILED'}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
