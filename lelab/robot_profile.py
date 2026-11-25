"""Which arm pair LeLab drives, selected with ``lelab --robot`` (or ``LELAB_ROBOT``).

``so101`` is the stock SO-101 leader/follower. ``dk1`` is the TRLC-DK1
(https://github.com/robot-learning-co/trlc-dk1), registered by the
``lerobot_robot_trlc_dk1`` plugin. The DK1 needs no calibration: its follower
uses absolute-encoder Damiao motors and its leader reads raw Dynamixel
positions, so the "calibration" files LeLab keeps for it are placeholders that
only record that an arm was found on a port.
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass, fields
from typing import Any

logger = logging.getLogger(__name__)

ROBOT_ENV_VAR = "LELAB_ROBOT"


@dataclass(frozen=True)
class RobotProfile:
    name: str
    follower_type: str
    leader_type: str
    # Directory names under LeRobot's calibration/{robots,teleoperators}/.
    follower_calibration_dir: str
    leader_calibration_dir: str
    # True for Feetech arms whose calibration LeLab writes to the motors itself.
    feetech_calibration: bool
    # Follower URDF served from frontend/public, shown in the 3D viewer.
    urdf_path: str
    # Two-arm variants taking left/right ports, or None if not supported.
    bimanual_follower_type: str | None = None
    bimanual_leader_type: str | None = None
    bimanual_urdf_path: str | None = None
    # Modules that must be imported for this profile's device types to appear in
    # LeRobot's config registry, which only fills in as device modules import.
    # Third-party devices (the DK1) register through the plugin hook instead.
    device_modules: tuple[str, ...] = ()


PROFILES = {
    "so101": RobotProfile(
        "so101",
        "so101_follower",
        "so101_leader",
        "so_follower",
        "so_leader",
        True,
        "/so-101-urdf/urdf/so101_new_calib.urdf",
        "bi_so_follower",
        "bi_so_leader",
        "/so-101-urdf/urdf/so101_new_calib-bimanual.urdf",
        (
            "lerobot.robots.so_follower",
            "lerobot.teleoperators.so_leader",
            "lerobot.robots.bi_so_follower",
            "lerobot.teleoperators.bi_so_leader",
        ),
    ),
    "dk1": RobotProfile(
        "dk1",
        "dk1_follower",
        "dk1_leader",
        "dk1_follower",
        "dk1_leader",
        False,
        "/dk1-urdf/TRLC-DK1-Follower.urdf",
        "bi_dk1_follower",
        "bi_dk1_leader",
        "/dk1-urdf/TRLC-DK1-Follower-bimanual.urdf",
    ),
}


def _load_profile() -> RobotProfile:
    name = os.environ.get(ROBOT_ENV_VAR, "so101").strip().lower()
    if name not in PROFILES:
        raise ValueError(f"{ROBOT_ENV_VAR}={name!r} is not one of {sorted(PROFILES)}")
    return PROFILES[name]


PROFILE = _load_profile()


def _register_devices() -> None:
    """Make this profile's device types resolvable by name."""
    import importlib

    for module in PROFILE.device_modules:
        importlib.import_module(module)
    if PROFILE.name != "so101":
        from lerobot.utils.import_utils import register_third_party_plugins

        register_third_party_plugins()


_register_devices()


def _bimanual_type(device_type: str | None) -> str:
    if device_type is None:
        raise ValueError(f"Bimanual mode is not supported for {PROFILE.name}")
    return device_type


def _bimanual_arm_kwargs(bimanual_cls, single_cls, left_port: str, right_port: str) -> dict[str, Any]:
    """How a bimanual config names its two arms, read off the config class itself.

    LeRobot's own pairs (bi_so_*) take a full single-arm config per side, while
    the DK1's take bare ports. Deriving this from the dataclass fields keeps the
    two styles from drifting out of a parallel table here.
    """
    names = {f.name for f in fields(bimanual_cls)}
    if {"left_arm_port", "right_arm_port"} <= names:
        return {"left_arm_port": left_port, "right_arm_port": right_port}
    if {"left_arm_config", "right_arm_config"} <= names:
        return {
            "left_arm_config": single_cls(port=left_port),
            "right_arm_config": single_cls(port=right_port),
        }
    raise ValueError(f"{bimanual_cls.__name__} does not take a left/right arm pair")


def follower_config(
    port: str, id: str | None = None, cameras: dict | None = None, right_port: str | None = None
):
    """Single-arm follower config, or the bimanual one when ``right_port`` is given
    (``port`` is then the left arm)."""
    from lerobot.robots import RobotConfig

    single_cls = RobotConfig.get_choice_class(PROFILE.follower_type)
    if right_port:
        cls = RobotConfig.get_choice_class(_bimanual_type(PROFILE.bimanual_follower_type))
        kwargs: dict[str, Any] = {
            "id": id,
            **_bimanual_arm_kwargs(cls, single_cls, port, right_port),
        }
    else:
        cls = single_cls
        kwargs = {"port": port, "id": id}
    if cameras is not None:
        # On a bimanual robot these stay top-level, so observation keys keep the
        # names the user gave them rather than gaining a left_/right_ prefix.
        kwargs["cameras"] = cameras
    return cls(**kwargs)


def leader_config(port: str, id: str | None = None, right_port: str | None = None):
    from lerobot.teleoperators import TeleoperatorConfig

    single_cls = TeleoperatorConfig.get_choice_class(PROFILE.leader_type)
    if right_port:
        cls = TeleoperatorConfig.get_choice_class(_bimanual_type(PROFILE.bimanual_leader_type))
        return cls(id=id, **_bimanual_arm_kwargs(cls, single_cls, port, right_port))
    return single_cls(port=port, id=id)


def make_follower(
    port: str, id: str | None = None, cameras: dict | None = None, right_port: str | None = None
):
    from lerobot.robots import make_robot_from_config

    return make_robot_from_config(follower_config(port, id, cameras, right_port))


def make_leader(port: str, id: str | None = None, right_port: str | None = None):
    from lerobot.teleoperators import make_teleoperator_from_config

    return make_teleoperator_from_config(leader_config(port, id, right_port))


def motor_buses(device) -> list:
    """The device's own bus, or both arms' buses for a bimanual device.

    A bimanual wrapper has no `.bus` of its own; it holds a `left_arm` and a
    `right_arm`, each a complete single-arm device with its own bus and
    calibration.
    """
    if hasattr(device, "bus"):
        return [device]
    arms = [getattr(device, side, None) for side in ("left_arm", "right_arm")]
    return [arm for arm in arms if arm is not None]


def _connect(label: str, port: str | None, fn) -> None:
    try:
        fn()
    except Exception as e:
        where = f" on {port}" if port else ""
        raise RuntimeError(
            f"Could not connect to the {label} arm{where}. "
            "Make sure it's plugged in and powered on, then try again."
        ) from e


def connect_devices(robot, teleop=None, follower_port: str | None = None, leader_port: str | None = None):
    """Bring up the follower (with its cameras) and optionally the leader.

    For the SO-101 this avoids ``robot.connect()``, which prompts on stdin when
    the motors disagree with the calibration file and leaves torque enabled
    before LeLab writes the calibration registers. The DK1 has no such prompt,
    so its own ``connect()`` is used as-is.
    """
    if not PROFILE.feetech_calibration:
        _connect("follower", follower_port, robot.connect)
        if teleop is not None:
            _connect("leader", leader_port, teleop.connect)
        return

    follower_arms = motor_buses(robot)
    leader_arms = motor_buses(teleop) if teleop is not None else []
    _connect("follower", follower_port, lambda: [arm.bus.connect() for arm in follower_arms])
    if teleop is not None:
        _connect("leader", leader_port, lambda: [arm.bus.connect() for arm in leader_arms])

    # Feetech motors only store calibration registers with the torque off,
    # which also opens their EEPROM lock.
    logger.info("Writing calibration to motors...")
    for arm in follower_arms + leader_arms:
        arm.bus.disable_torque()
        arm.bus.write_calibration(arm.calibration)

    for cam in robot.cameras.values():
        cam.connect()
    robot.configure()
    if teleop is not None:
        teleop.configure()
