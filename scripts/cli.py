"""CLI entrypoints for building and running the quadrotor simulator."""

import argparse
import os
import signal
import subprocess
import sys
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
BUILD_DIR = PROJECT_ROOT / "build_standalone"
CORE_BIN = BUILD_DIR / "quadrotor_sim_core"
GLFW_BIN = BUILD_DIR / "quadrotor_sim_glfw_adapter"
MODEL_DEFAULT = "deps/model/mujoco/drone.xml"
CMAKE_STANDALONE = PROJECT_ROOT / "CMakeLists_standalone.txt"


def _env():
    env = os.environ.copy()
    lib_path = str(PROJECT_ROOT / "deps" / "lib")
    existing = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = f"{lib_path}:{existing}" if existing else lib_path
    return env


def _ensure_built():
    """Build C++ binaries if not already present."""
    if CORE_BIN.is_file():
        return

    import shutil

    if BUILD_DIR.exists():
        shutil.rmtree(BUILD_DIR)
    BUILD_DIR.mkdir(parents=True, exist_ok=True)

    # Create a trampoline CMakeLists.txt in a temp dir that includes the
    # standalone file.  cmake reads CMakeLists.txt from the -S directory.
    with tempfile.TemporaryDirectory(prefix="sim_build_") as tmp_src:
        trampoline = Path(tmp_src) / "CMakeLists.txt"
        with open(trampoline, "w") as f:
            f.write("cmake_minimum_required(VERSION 3.10)\n")
            f.write(f'include("{CMAKE_STANDALONE}")\n')

        subprocess.run(
            [
                "cmake",
                "-Wno-dev",
                "-DCMAKE_BUILD_TYPE=Release",
                "-S",
                tmp_src,
                "-B",
                str(BUILD_DIR),
            ],
            check=True,
            env=_env(),
        )
        subprocess.run(
            ["cmake", "--build", str(BUILD_DIR), "--parallel"],
            check=True,
            env=_env(),
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
    os.execve(str(CORE_BIN), argv, _env())


def run_glfw(args: argparse.Namespace) -> int:
    """Start core in background, then run GLFW viewer in foreground.

    Core is automatically killed when GLFW exits or Ctrl+C is received.
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

    p_glfw = sub.add_parser("glfw", help="Start GLFW render viewer")
    p_glfw.add_argument("--model", help=f"Path to drone.xml (default: {MODEL_DEFAULT})")
    p_glfw.set_defaults(func=run_glfw)

    args = parser.parse_args()
    sys.exit(args.func(args) or 0)
