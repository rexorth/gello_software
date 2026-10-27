import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import tyro

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

MENAGERIE_ROOT: Path = Path(__file__).parent / "third_party" / "mujoco_menagerie"


@dataclass
class Args:
    port: str = "/dev/ttyUSB0"
    """The port that GELLO is connected to."""

    start_joints: Tuple[float, ...] = (0, 0, 0, 0, 0, 0)
    """The joint angles that the GELLO is placed in at (in radians)."""

    joint_signs: Tuple[float, ...] = (1, 1, -1, 1, 1, 1)
    """The joint angles that the GELLO is placed in at (in radians)."""

    gripper: bool = True
    """Whether or not the gripper is attached."""

    encoder: bool = False
    """GELLO built with AS5600 magnetic encoders (firmware/as5600_leader) instead of Dynamixels:
    offsets come out exact, not multiples of pi/2, and the gripper is measured open and closed."""

    channels: Tuple[int, ...] = (0, 1, 2, 3, 4, 5)
    """--encoder: the multiplexer channel of each joint, base first."""

    gripper_channel: int = 6
    """--encoder: the multiplexer channel of the trigger's encoder."""

    watch: bool = False
    """--encoder: show the joints live instead, through --joint-signs and --joint-offsets, to
    check that each one turns the way the robot's does."""

    joint_offsets: Optional[Tuple[float, ...]] = None
    """--encoder --watch: the offsets to show the joints through (radians); default none."""

    def __post_init__(self):
        assert len(self.joint_signs) == len(self.start_joints)
        for idx, j in enumerate(self.joint_signs):
            assert (
                j == -1 or j == 1
            ), f"Joint idx: {idx} should be -1 or 1, but got {j}."

    @property
    def num_robot_joints(self) -> int:
        return len(self.start_joints)

    @property
    def num_joints(self) -> int:
        extra_joints = 1 if self.gripper else 0
        return self.num_robot_joints + extra_joints


def get_config(args: Args) -> None:
    from gello.dynamixel.driver import DynamixelDriver

    joint_ids = list(range(1, args.num_joints + 1))
    driver = DynamixelDriver(joint_ids, port=args.port, baudrate=57600)

    # assume that the joint state shouold be args.start_joints
    # find the offset, which is a multiple of np.pi/2 that minimizes the error between the current joint state and args.start_joints
    # this is done by brute force, we seach in a range of +/- 8pi

    def get_error(offset: float, index: int, joint_state: np.ndarray) -> float:
        joint_sign_i = args.joint_signs[index]
        joint_i = joint_sign_i * (joint_state[index] - offset)
        start_i = args.start_joints[index]
        return np.abs(joint_i - start_i)

    for _ in range(10):
        driver.get_joints()  # warmup

    for _ in range(1):
        best_offsets = []
        curr_joints = driver.get_joints()
        for i in range(args.num_robot_joints):
            best_offset = 0
            best_error = 1e6
            for offset in np.linspace(
                -8 * np.pi, 8 * np.pi, 8 * 4 + 1
            ):  # intervals of pi/2
                error = get_error(offset, i, curr_joints)
                if error < best_error:
                    best_error = error
                    best_offset = offset
            best_offsets.append(best_offset)
        print()
        print("best offsets               : ", [f"{x:.3f}" for x in best_offsets])
        print(
            "best offsets function of pi: ["
            + ", ".join([f"{int(np.round(x/(np.pi/2)))}*np.pi/2" for x in best_offsets])
            + " ]",
        )
        if args.gripper:
            print(
                "gripper open (degrees)       ",
                np.rad2deg(driver.get_joints()[-1]) - 0.2,
            )
            print(
                "gripper close (degrees)      ",
                np.rad2deg(driver.get_joints()[-1]) - 42,
            )


def _encoder_driver(args: Args, with_gripper: bool):
    from gello.encoder import EncoderDriver

    assert (
        len(args.channels) == args.num_robot_joints
    ), f"--channels has {len(args.channels)} channels for {args.num_robot_joints} joints"
    channels = list(args.channels) + ([args.gripper_channel] if with_gripper else [])
    return EncoderDriver(channels, port=args.port)


def _mean_joints(driver, seconds: float = 0.25) -> np.ndarray:
    """The joints averaged over a moment (raw radians, gripper last)."""
    samples = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        samples.append(driver.get_joints())
        time.sleep(0.005)
    return np.mean(samples, axis=0)


def get_encoder_config(args: Args) -> None:
    """Offsets for a GELLO with magnetic encoders. Each magnet sits at an arbitrary angle, so an
    offset is whatever puts the joint at start_joints in the calibration pose: it is exact, with
    no rounding to pi/2. Whole turns don't matter: DynamixelRobot resolves them against
    start_joints at startup."""
    driver = _encoder_driver(args, with_gripper=False)
    try:
        joints = _mean_joints(driver)
    finally:
        driver.close()
    signs = np.array(args.joint_signs)
    offsets = joints - signs * np.array(args.start_joints)
    print()
    print("joint offsets (rad): ", [f"{x:.4f}" for x in offsets])
    print("joint offsets (deg): ", [f"{x:.2f}" for x in np.rad2deg(offsets)])

    gripper = None
    if args.gripper:
        driver = _encoder_driver(args, with_gripper=True)
        try:
            input("\nRelease the trigger fully (gripper open), then press Enter ")
            open_rad = _mean_joints(driver)[-1]
            input("Squeeze the trigger fully (gripper closed), then press Enter ")
            closed_rad = _mean_joints(driver)[-1]
        finally:
            driver.close()
        # The trigger moves less than half a turn: the short way round from open to closed.
        closed_rad = open_rad + (closed_rad - open_rad + np.pi) % (2 * np.pi) - np.pi
        gripper = (
            args.gripper_channel,
            round(np.rad2deg(open_rad), 1),
            round(np.rad2deg(closed_rad), 1),
        )
        print(f"gripper (channel, open deg, closed deg): {gripper}")

    print("\nFor PORT_CONFIG_MAP in gello/agents/gello_agent.py:")
    print(f'    "{args.port}": EncoderRobotConfig(')
    print(f"        channels={tuple(args.channels)},")
    print(f"        joint_offsets={tuple(round(float(x), 4) for x in offsets)},")
    print(f"        joint_signs={tuple(int(x) for x in args.joint_signs)},")
    print(f"        gripper_config={gripper},")
    print("    ),")


def watch_encoders(args: Args) -> None:
    """The joints live, as the robot would see them with these signs and offsets: move each
    joint the way the robot's turns positive and its value should rise."""
    driver = _encoder_driver(args, with_gripper=args.gripper)
    n = args.num_robot_joints
    offsets = (
        np.zeros(n) if args.joint_offsets is None else np.array(args.joint_offsets)
    )
    signs = np.array(args.joint_signs)
    print(
        "joint degrees (Ctrl-C to stop)"
        + (", then the trigger's raw degrees" if args.gripper else "")
    )
    try:
        while True:
            q = driver.get_joints()
            joints = (q[:n] - offsets) * signs
            text = " ".join(f"{v:8.1f}" for v in np.rad2deg(joints))
            if args.gripper:
                text += f" | {np.rad2deg(q[-1]):7.1f}"
            print("\r" + text, end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        driver.close()


def main(args: Args) -> None:
    if args.encoder and args.watch:
        watch_encoders(args)
    elif args.encoder:
        get_encoder_config(args)
    else:
        get_config(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
