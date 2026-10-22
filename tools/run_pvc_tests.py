"""Test real portable modules and hardware-gated chassis at all four stages."""

from pathlib import Path
import shutil
import subprocess


def main():
    """Compile protocol, mission, slope, and chassis against actuator mocks."""
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("gcc")

    if compiler is None:
        raise SystemExit("gcc was not found on PATH")

    build = root / "Debug/pvc_tests"
    build.mkdir(parents=True, exist_ok=True)
    includes = [
        "tests/mock", "application", "application/chassis", "application/race",
        "modules/vision", "modules/slope", "modules/motor",
    ]
    sources = [
        "tests/pvc_test.c", "modules/vision/vision.c", "modules/slope/slope.c",
        "application/race/race_control.c",
        "application/chassis/chassis_control.c",
        "application/tuning/car_tuning.c",
    ]
    stage = 0
    executable = None
    command = []

    for stage in range(1, 5):
        executable = build / f"pvc_stage_{stage}.exe"
        command = [
            compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-fstack-protector-all", f"-DCAR_STAGE={stage}",
        ]
        command += [f"-I{root / item}" for item in includes]
        command += [str(root / item) for item in sources]
        command += ["-lm", "-o", str(executable)]
        subprocess.run(command, cwd=root, check=True)
        subprocess.run([str(executable)], cwd=root, check=True, timeout=10)


if __name__ == "__main__":
    main()
