"""Test real application input gates with mocked hardware and menu intents."""
from pathlib import Path
import shutil
import subprocess


def main():
    """Check four menu stages and the existing headless key behavior."""
    root = Path(__file__).resolve().parents[1]
    build = root / "Debug/car_app_tests"
    build.mkdir(parents=True, exist_ok=True)
    includes = [
        "tests/mock/car_app", "tests/mock", ".", "application",
        "application/menu", "application/race", "application/chassis",
        "modules/vision", "modules/slope", "modules/imu", "modules/key",
        "bsp/uart",
    ]
    sources = [
        "tests/car_app_test.c", "application/car_app.c",
        "application/race/race_control.c", "modules/vision/vision.c",
        "modules/slope/slope.c",
    ]
    compiler = shutil.which("gcc")
    assert compiler is not None, "gcc was not found on PATH"
    stage = 0
    menu = 0
    executable = None
    command = []

    for stage, menu in ((1, 1), (2, 1), (3, 1), (4, 1), (3, 0)):
        executable = build / f"stage_{stage}_menu_{menu}.exe"
        command = [
            compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-Wno-unused-parameter",
            f"-DCAR_STAGE={stage}", f"-DCAR_ENABLE_MENU={menu}",
        ]
        command += [f"-I{root / item}" for item in includes]
        command += [str(root / item) for item in sources]
        command += ["-lm", "-o", str(executable)]
        subprocess.run(command, check=True, cwd=root)
        subprocess.run([str(executable)], check=True, cwd=root, timeout=10)


if __name__ == "__main__":
    main()
