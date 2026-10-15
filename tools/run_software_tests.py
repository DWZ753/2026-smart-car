"""Compile and run driver regression tests on the host, without hardware."""

from pathlib import Path
import shutil
import subprocess


def main():
    """Build real driver sources against mock registers and run assertions."""
    root = Path(__file__).resolve().parents[1]
    compiler = shutil.which("gcc")

    if compiler is None:
        raise SystemExit("gcc was not found on PATH")

    build_dir = root / "Debug" / "software_tests"
    build_dir.mkdir(parents=True, exist_ok=True)
    executable = build_dir / "software_test.exe"
    include_dirs = [
        "tests/mock", "bsp/uart", "bsp/spi", "bsp/i2c", "bsp/delay",
        "modules/protocol", "modules/algorithm/pid",
    ]
    sources = [
        "tests/software_test.c", "bsp/uart/uart.c", "bsp/spi/spi.c",
        "bsp/i2c/i2c.c", "modules/protocol/protocol.c",
        "modules/protocol/cobs.c", "modules/algorithm/pid/pid.c",
    ]
    command = [
        compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-Wno-unused-parameter", "-Wno-pointer-to-int-cast",
        "-fstack-protector-all",
    ]
    command += [f"-I{root / directory}" for directory in include_dirs]
    command += [str(root / source) for source in sources]
    command += ["-o", str(executable)]
    subprocess.run(command, check=True, cwd=root)
    subprocess.run([str(executable)], check=True, timeout=10, cwd=root)


if __name__ == "__main__":
    main()
