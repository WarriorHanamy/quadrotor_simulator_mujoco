"""CLI entrypoints for building and running the quadrotor simulator."""

import argparse
import os
import signal
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
BUILD_DIR = PROJECT_ROOT / "build_standalone"
CORE_BIN = BUILD_DIR / "quadrotor_sim_core"
GLFW_BIN = BUILD_DIR / "quadrotor_sim_glfw_adapter"
SE3_BIN = BUILD_DIR / "quadrotor_sim_se3_direct"
MODEL_DEFAULT = "deps/model/mujoco/drone.xml"
GAINS_DEFAULT = "config/se3_gains.yaml"


def _env():
    env = os.environ.copy()
    lib_path = str(PROJECT_ROOT / "deps" / "lib")
    existing = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = f"{lib_path}:{existing}" if existing else lib_path
    return env


def _ensure_built():
    """Build C++ binaries if not already present."""
    if CORE_BIN.is_file() and SE3_BIN.is_file():
        print("sim: build skipped — binaries already up to date")
        return

    print("sim: building via xmake...")
    subprocess.run(
        ["xmake", "build", "-w", "-j"],
        check=True,
        env=_env(),
        cwd=str(PROJECT_ROOT),
    )


def run_core(args: argparse.Namespace) -> int:
    _ensure_built()
    model = args.model or str(PROJECT_ROOT / MODEL_DEFAULT)
    argv = [str(CORE_BIN), model]
    if args.real_time_factor is not None:
        argv += ["--real-time-factor", str(args.real_time_factor)]
    if args.ctrlnoise_std is not None:
        argv += ["--ctrlnoise-std", str(args.ctrlnoise_std)]
    if args.ctrlnoise_rate is not None:
        argv += ["--ctrlnoise-rate", str(args.ctrlnoise_rate)]
    p = subprocess.Popen(argv, env=_env())
    return p.wait()


def run_render(args: argparse.Namespace) -> int:
    """Start core in background, then run render viewer in foreground.

    Core is automatically killed when viewer exits or Ctrl+C is received.
    """
    _ensure_built()
    model = args.model or str(PROJECT_ROOT / MODEL_DEFAULT)

    core_proc: subprocess.Popen | None = None

    def _on_signal(sig: int, _frame):
        if core_proc:
            core_proc.terminate()
        sys.exit(128 + sig)

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        core_proc = subprocess.Popen(
            [str(CORE_BIN), model],
            env=_env(),
        )
        glfw_proc = subprocess.Popen(
            [str(GLFW_BIN), model],
            env=_env(),
        )
        return glfw_proc.wait()
    finally:
        if core_proc:
            core_proc.terminate()
            try:
                core_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                core_proc.kill()
                core_proc.wait()


def run_se3(args: argparse.Namespace) -> int:
    """Start core + SE(3) controller, optionally with GLFW render viewer.

    Core runs in background; controller runs in foreground (Ctrl-C stops both).
    """
    _ensure_built()
    model = args.model or str(PROJECT_ROOT / MODEL_DEFAULT)

    core_proc: subprocess.Popen | None = None
    glfw_proc: subprocess.Popen | None = None

    def _on_signal(sig: int, _frame):
        if glfw_proc:
            glfw_proc.terminate()
        if core_proc:
            core_proc.terminate()
        sys.exit(128 + sig)

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    # Build SE3 controller argv
    gains_path = args.gains_file or str(PROJECT_ROOT / GAINS_DEFAULT)
    se3_argv = [
        str(SE3_BIN),
        "--pos-x",
        str(args.pos_x),
        "--pos-y",
        str(args.pos_y),
        "--pos-z",
        str(args.pos_z),
        "--yaw",
        str(args.yaw),
        "--gains-file",
        gains_path,
    ]
    if args.rate:
        se3_argv += ["--rate", str(args.rate)]

    try:
        core_proc = subprocess.Popen(
            [str(CORE_BIN), model], env=_env(), start_new_session=True
        )
        if args.render:
            glfw_proc = subprocess.Popen(
                [str(GLFW_BIN), model], env=_env(), start_new_session=True
            )
        # Run SE3 controller as foreground child (not execve —
        # execve would orphan core/glfw and send them stray signals)
        se3_proc = subprocess.Popen(se3_argv, env=_env())
        se3_proc.wait()
    finally:
        if glfw_proc:
            glfw_proc.terminate()
            try:
                glfw_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                glfw_proc.kill()
                glfw_proc.wait()
        if core_proc:
            core_proc.terminate()
            try:
                core_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                core_proc.kill()
                core_proc.wait()


def main() -> None:
    parser = argparse.ArgumentParser(prog="sim")
    sub = parser.add_subparsers(dest="command", required=True)

    p_build = sub.add_parser("build", help="Compile C++ binaries")
    p_build.set_defaults(func=lambda _: _ensure_built() or 0)

    p_run = sub.add_parser("run", help="Start headless core simulator")
    p_run.add_argument("--model", help=f"Path to drone.xml (default: {MODEL_DEFAULT})")
    p_run.add_argument("--real-time-factor", type=float, help="1.0 = real-time")
    p_run.add_argument("--ctrlnoise-std", type=float, help="Control noise std")
    p_run.add_argument("--ctrlnoise-rate", type=float, help="Control noise rate")
    p_run.set_defaults(func=run_core)

    p_render = sub.add_parser("render", help="Start render viewer")
    p_render.add_argument(
        "--model", help=f"Path to drone.xml (default: {MODEL_DEFAULT})"
    )
    p_render.set_defaults(func=run_render)

    p_se3 = sub.add_parser("run-se3", help="Start core + SE(3) controller")
    p_se3.add_argument("--model", help=f"Path to drone.xml (default: {MODEL_DEFAULT})")
    p_se3.add_argument("--pos-x", type=float, default=0.0, help="Setpoint X [m]")
    p_se3.add_argument("--pos-y", type=float, default=0.0, help="Setpoint Y [m]")
    p_se3.add_argument("--pos-z", type=float, default=2.0, help="Setpoint Z [m]")
    p_se3.add_argument("--yaw", type=float, default=0.0, help="Setpoint yaw [rad]")
    p_se3.add_argument(
        "--rate", type=float, default=500.0, help="Control loop rate [Hz]"
    )
    p_se3.add_argument("--render", action="store_true", help="Also start GLFW viewer")
    p_se3.add_argument(
        "--gains-file", help=f"Path to se3_gains.yaml (default: {GAINS_DEFAULT})"
    )
    p_se3.set_defaults(func=run_se3)

    args = parser.parse_args()
    sys.exit(args.func(args) or 0)
