<h1 align="center">🦾 LeLab, bimanual and beyond the SO-101</h1>

<p align="center">
  A fork of <a href="https://github.com/huggingface/leLab">huggingface/leLab</a> that drives
  <a href="https://github.com/robot-learning-co/trlc-dk1">TRLC-DK1</a> arms as well as SO-101s,
  and either of them as a <b>bimanual</b> four-arm rig.
</p>

Upstream LeLab is hardcoded for a single SO-101 leader/follower pair. This fork adds a
**robot profile** layer, so the same web UI — calibrate, teleoperate, record, replay,
train, run inference — works with:

- a **single SO-101** pair, exactly as upstream (the default);
- a **bimanual SO-101** rig, two leaders and two followers as one robot;
- a **single or bimanual TRLC-DK1** rig, with `--robot dk1`;

each with as many cameras as you can plug in. For general LeLab documentation see
[README.upstream.md](README.upstream.md).

> **Status:** exercised end-to-end against a simulated rig (see
> [Testing without hardware](#testing-without-hardware)) and, for a single SO-101 pair,
> on real arms. **Bimanual has not yet run on real hardware** for either robot.
> Read [Safety](#safety) before your first run.

---

## What this fork adds

| | |
|---|---|
| 👐 **Bimanual** | Pair two robot configs into one four-arm rig — `bi_so_follower` for SO-101s, `bi_dk1_follower` for DK1s. Datasets get `left_*` / `right_*` joint names. |
| 🤖 **DK1 support** | `lelab --robot dk1` drives `dk1_follower` / `dk1_leader` everywhere: teleop, recording, replay and inference. |
| 🧊 **3D viewer** | Loads the follower model of whichever robot the server drives, and a bimanual rig shows **both** arms moving live. |
| 📷 **Any number of cameras** | All configured cameras stream side by side while recording (tested with four). |
| 🚫 **No calibration dance (DK1)** | The DK1 uses absolute encoders, so "Calibrate" just confirms the arm answers on its port. SO-101s calibrate as usual, per arm. |
| 🧪 **Fake hardware rig** | Run and verify the whole app — including dataset fidelity — with no arms or cameras attached. |

---

## Requirements

- **Linux** (developed on Ubuntu 22.04; macOS/Windows untested with the DK1)
- **Python 3.12** — LeLab pins LeRobot `v0.7.0`, which needs it
- Serial access to the arms: your user must be in the `dialout` group
  ```bash
  groups | grep -q dialout || sudo usermod -aG dialout "$USER"   # then log out and back in
  ```
- Network access on first install (it downloads LeRobot + PyTorch and builds the web UI)

## Install

```bash
python3.12 -m venv ~/lelab-dk1 && ~/lelab-dk1/bin/pip install -U pip
```

```bash
~/lelab-dk1/bin/pip install git+https://github.com/muayad-chalabi/leLab.git@dk1 git+https://github.com/muayad-chalabi/trlc-dk1.git
```

That is the whole install: it pulls LeRobot v0.7.0, builds the web UI (Node comes from a
build-time wheel — you do **not** need Node installed), and registers the DK1 plugin so
LeRobot discovers `dk1_follower`, `dk1_leader`, `bi_dk1_follower` and `bi_dk1_leader`.

The DK1 package is installed from **this fork** of `trlc-dk1` because upstream's
bimanual follower crashes on creation; see [Relationship to upstream](#relationship-to-upstream).

Verify the plugin registered:

```bash
~/lelab-dk1/bin/python -c "from lerobot.utils.import_utils import register_third_party_plugins as r; r(); from lerobot.robots import RobotConfig; print(sorted(k for k in RobotConfig.get_known_choices() if 'dk1' in k))"
```

Expect `['bi_dk1_follower', 'dk1_follower']`.

> **Using an existing LeRobot checkout?** Install the plugin into that environment with
> `pip install -e /path/to/trlc-dk1` instead. The DK1 package needs Python ≥3.10 (this
> fork relaxed it from 3.12), `dynamixel-sdk`, `pyserial` and `mujoco`.

## Run

```bash
~/lelab-dk1/bin/lelab --robot dk1
```

The UI opens at <http://127.0.0.1:8000>. Drop `--robot dk1` to drive SO-101s instead
(the default). The choice can also come from the `LELAB_ROBOT` environment variable.
Bimanual works in either mode — see [Bimanual](#bimanual-four-arms-as-one-robot).

## First run: one arm pair

1. **Find your ports.** With everything unplugged, plug in one controller at a time:
   ```bash
   ~/lelab-dk1/bin/lerobot-find-port
   ```
2. **Add a robot** on the landing page (e.g. `left`), then press the gear icon to configure it.
3. **"Calibrate" each side.** SO-101s run the usual guided range-of-motion flow. On
   the DK1 this only connects to the arm, confirms it answers on that port and saves a
   placeholder file — there is no range-of-motion routine. Either way, support the
   follower: it goes limp when the step ends.
4. **Teleoperate.** Match the leader to the follower's pose first (see [Safety](#safety)),
   then press **Teleoperation**.

## Bimanual: four arms as one robot

1. Create **two** robots, e.g. `left` and `right`, and configure both as above —
   each one owns its own leader and follower ports.
2. On the `left` robot's card, set **Right arm** to `right`.
3. Press **Teleoperation**. LeLab now builds one four-arm robot from all four ports:
   `bi_so_follower` / `bi_so_leader` for SO-101s, `bi_dk1_follower` / `bi_dk1_leader`
   for DK1s.

Notes:

- The pair only shows **Ready** when *both* robots are configured.
- Cameras come from the **left** (primary) robot's camera list — add every camera there,
  not split across the two. Their names stay as you typed them (no `left_`/`right_`
  prefix), so dataset image keys match a single-arm recording.
- Recordings use `left_*` / `right_*` joint names, the same layout the LeRobot CLI
  produces for these robots, so datasets stay compatible with upstream tooling.
- **SO-101 calibration:** each arm keeps the calibration you recorded for it. LeLab
  republishes the two files under the paired names LeRobot expects
  (`<left>__<right>_left.json` and `..._right.json`), refreshed on every run, so
  recalibrating either arm takes effect immediately.
- Setting **Right arm** back to *None (single arm)* restores single-arm mode.
- Replay and inference send both arms too.

## Cameras

Add cameras to a robot on the Calibration page. During recording every configured
camera is shown live, side by side, each labelled with its name. Find camera indices with:

```bash
~/lelab-dk1/bin/lerobot-find-cameras
```

## Known issues

### Turn off streaming video encoding when recording

**In the recording dialog, open *Advanced Parameters* and uncheck *Streaming video encoding*.**

With it on (LeLab's default), the start of each episode opens one video encoder per
camera, which stalls the whole process for roughly 270 ms. A camera read times out
after 200 ms, so the session dies with `no new frame in 200 ms`. With four simulated
cameras this failed in 3 out of 3 attempts; with streaming off the worst stall was
72 ms and nothing failed. This was measured with fake cameras — real cameras share the
same bottleneck (the Python GIL), so expect the same.

The cost of turning it off is a pause after each episode while its videos encode:
about 5 s for an 8 s episode with four cameras. The follower holds its pose meanwhile.

### An episode is only saved if you press "End Episode"

This is upstream LeLab behaviour, not something this fork introduced. If the episode
timer runs out first, the take is **discarded and re-recorded**. Treat the episode
duration as a maximum and press **End Episode** (or <kbd>Space</kbd>) to keep a take.

### The 3D viewer starts zoomed to the folded pose

When the arm stretches out it can leave the frame. Scroll to zoom out.

---

## Safety

The control logic is the DK1 plugin's own — identical to what
`lerobot-teleoperate` runs from a terminal — so LeLab is no more aggressive than the
CLI. The things to know apply to both:

- **No ramp on the first command.** The first leader reading becomes the follower's
  target at full stiffness. **Match each leader to its follower's pose before pressing
  Start**, or the follower will snap across.
- **Stopping releases the motors.** Stop, closing the server, or any error in the loop
  disables the follower's motors: the arm goes limp and drops. Support it, or park it
  in a safe pose first. This includes the "Calibrate" step.
- **Bimanual doubles this.** Both followers go limp together.
- The DK1's own limits still apply: targets are clipped to the joint limits, torque is
  capped, and sustained over-current drops the arm into damping mode.
- Keep the e-stop or power switch in reach for the first runs.

A good first hardware test: run one arm pair from the terminal
(`lerobot-teleoperate --robot.type=dk1_follower ...`) to confirm the arms behave, then
repeat it in LeLab with a hand ready to catch the follower.

---

## Testing without hardware

`tests/fake_hw/` replaces the DK1 motor stack and OpenCV cameras in-process, so the
**unmodified** app runs end to end with no arms or cameras attached. Each fake camera
frame carries its camera index and capture time encoded as black/white blocks, which
lets a checker prove the recorded dataset really contains what the robot produced.

Always use a throwaway `HOME` so it cannot touch your real robots or datasets:

```bash
export FAKE=/tmp/lelab-fake && rm -rf $FAKE && mkdir -p $FAKE
```

```bash
HOME=$FAKE python tests/fake_hw/setup_fake_robots.py
```

```bash
HOME=$FAKE FAKE_HW_LOG=$FAKE/hw.jsonl python tests/fake_hw/run_lelab.py --port 8010
```

Open <http://127.0.0.1:8010/?api=http://127.0.0.1:8010> and drive it by hand, or record
a dataset through the API the way the UI does:

```bash
python tests/fake_hw/record_via_api.py --api http://127.0.0.1:8010 --episodes 2 --episode-s 8 --no-streaming
```

Then check the result against what the fake hardware actually produced:

```bash
HOME=$FAKE python tests/fake_hw/check_dataset.py $FAKE/.cache/huggingface/lerobot/local/<dataset> $FAKE/hw.jsonl --cameras front=1,top=2,left_wrist=3,right_wrist=4 --episodes 2 --episode-s 8
```

It verifies metadata and feature order, episode/frame indexing and timestamps, that
every action and state row matches the logged hardware values in order (left and right
not swapped), that `state[k] == action[k-1]`, and that every decoded video frame is the
exact frame that camera returned for that row.

The `--port` flag keeps this rig on its own port, so it can run beside a real LeLab on
`:8000`.

### Project tests

```bash
pytest                    # backend
cd frontend && npm test   # frontend
```

If you have ROS on your `PATH`, its plugins break pytest collection — run
`env -u PYTHONPATH pytest`.

---

## Relationship to upstream

This fork tracks [huggingface/leLab](https://github.com/huggingface/leLab) on `main`;
the work lives on the **`dk1`** branch (this repo's default).

**DK1 plugin fix.** `BiDK1Follower` read a `.motors` attribute that `DK1Follower` does
not have, so creating a `bi_dk1_follower` raised `AttributeError` before it could
connect — bimanual mode was broken from the LeRobot CLI too, not just in LeLab.
[Our trlc-dk1 fork](https://github.com/muayad-chalabi/trlc-dk1) fixes that and relaxes
the Python pin to ≥3.10. It has not been proposed upstream yet.

**Where the changes live:**

| Path | Purpose |
|---|---|
| [`lelab/robot_profile.py`](lelab/robot_profile.py) | The profile layer: which device types, calibration directories and URDFs a robot uses, plus device construction and connection. |
| [`lelab/teleoperate.py`](lelab/teleoperate.py) | Observation → URDF joint mapping for the live 3D view, single and bimanual, for both robots. |
| [`lelab/utils/config.py`](lelab/utils/config.py) | Calibration files, including the paired `<left>__<right>_{left,right}.json` a bimanual SO-101 reads. |
| [`frontend/public/dk1-urdf/`](frontend/public/dk1-urdf/) | DK1 follower URDF + meshes, vendored from trlc-dk1 ([SOURCE.md](frontend/public/dk1-urdf/SOURCE.md)), plus the generated two-arm model. |
| [`scripts/make_bimanual_urdf.py`](scripts/make_bimanual_urdf.py) | Regenerates each two-arm URDF from its single-arm one. |
| [`tests/fake_hw/`](tests/fake_hw/) | The simulated DK1 rig described above. |

Adding another robot means adding one entry to `PROFILES` in `lelab/robot_profile.py`
and a URDF — the feature modules no longer name a robot type. A profile lists the
modules that register its LeRobot device types, since that registry only fills in as
device modules are imported.

## License

Apache 2.0, as upstream. The vendored DK1 URDF and meshes are Apache 2.0 from
The Robot Learning Company UG — see
[`frontend/public/dk1-urdf/SOURCE.md`](frontend/public/dk1-urdf/SOURCE.md).
