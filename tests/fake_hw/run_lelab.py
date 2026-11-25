"""Run LeLab against fake DK1 arms and cameras (see fake_dk1.py).

    FAKE_HW_LOG=/tmp/fake_hw.jsonl python tests/fake_hw/run_lelab.py --no-open
    FAKE_HW_LOG=/tmp/fake_hw.jsonl python tests/fake_hw/run_lelab.py --port 8010

With --port it serves the API and built frontend on that port only (no browser,
no dev mode), so it can run beside a real LeLab on :8000; open
http://127.0.0.1:<port>/?api=http://127.0.0.1:<port>.

Any port works; leader ports containing "right" get the right-arm trajectory.
Cameras are fake for every `camera_index`. Run it with a throwaway HOME so the
robots, calibrations and datasets it creates stay out of your real cache.
"""

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
os.environ["LELAB_ROBOT"] = "dk1"

import fake_dk1  # noqa: E402

from lelab.robot_profile import PROFILE  # noqa: E402  (registers the DK1 plugin first)

assert PROFILE.name == "dk1"
fake_dk1.install(os.environ.get("FAKE_HW_LOG"))

from lelab.scripts.lelab import main  # noqa: E402

if __name__ == "__main__":
    args = sys.argv[1:]
    if "--port" in args:
        import uvicorn

        port = int(args[args.index("--port") + 1])
        uvicorn.run("lelab.server:app", host="127.0.0.1", port=port)
    else:
        main(["--robot", "dk1", *args])
