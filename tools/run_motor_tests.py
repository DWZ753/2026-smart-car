"""Test real motor conversion and initialization with driver mocks."""

from pathlib import Path
import shutil
import subprocess


def main():
    """Verify the module boundary without driving physical motors."""
    root = Path(__file__).resolve().parents[1]
    build = root / "Debug/motor_tests"
    build.mkdir(parents=True, exist_ok=True)
    compiler = shutil.which("gcc")
    assert compiler is not None, "gcc was not found on PATH"
    output = build / "motor_test.exe"
    includes = [
        "tests/mock/motor", "tests/mock", "modules/motor",
        "modules/pit_tick",
    ]
    command = [
        compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-Wno-unused-parameter",
    ]
    command += [f"-I{root / path}" for path in includes]
    command += [str(root / path) for path in (
        "tests/motor_test.c", "modules/motor/motor.c")]
    command += ["-lm", "-o", str(output)]
    subprocess.run(command, cwd=root, check=True)
    subprocess.run([str(output)], cwd=root, check=True, timeout=10)


if __name__ == "__main__":
    main()
