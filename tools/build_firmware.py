"""Generate SysConfig and compile CCS firmware at selected CAR_STAGE values."""

import argparse
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ElementTree


ROOT = Path(__file__).resolve().parents[1]
SDK = Path("E:/ti/ccs2050/mspm0_sdk_2_11_00_07")
COMPILER = Path(
    "E:/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/"
    "bin/tiarmclang.exe"
)
SYSCONFIG = Path("E:/ti/ccs2050/sysconfig_1.26.2/sysconfig_cli.bat")
BUILD = ROOT / "Debug/firmware_build"


def run(command, directory):
    """Capture diagnostics in the build folder and fail on compiler errors."""
    result = subprocess.run(
        list(map(str, command)), cwd=directory, capture_output=True,
        text=True, errors="replace", check=False,
    )
    output = result.stdout + result.stderr

    with (directory / "build.log").open("a", encoding="utf-8") as log:
        log.write(output)

    if output:
        print(output, end="")

    result.check_returncode()


def build_stage(stage, generated):
    """Apply CCS source exclusions and link a complete selected firmware."""
    directory = BUILD / (f"stage_{stage}" if stage else "default")
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "build.log").write_text("", encoding="utf-8")
    project = ElementTree.parse(ROOT / ".cproject")
    excluded = project.find(".//sourceEntries/entry").get("excluding")
    excluded = excluded.split("|")
    includes = [ROOT, generated, SDK / "source", SDK / (
        "source/third_party/CMSIS/Core/Include")]
    includes += sorted({
        path.parent for group in ("bsp", "modules", "application")
        for path in (ROOT / group).rglob("*.h")
    })
    sources = sorted(ROOT.glob("*.c"))
    sources += sorted(path for group in ("bsp", "modules", "application")
                      for path in (ROOT / group).rglob("*.c"))
    sources = [path for path in sources if not any(
        path.relative_to(ROOT).as_posix() == item or
        path.relative_to(ROOT).as_posix().startswith(item + "/")
        for item in excluded)]
    sources += [generated / "ti_msp_dl_config.c", SDK / (
        "source/ti/devices/msp/m0p/startup_system_files/ticlang/"
        "startup_mspm0g350x_ticlang.c")]
    flags = [
        "-march=thumbv6m", "-mcpu=cortex-m0plus", "-mfloat-abi=soft",
        "-mlittle-endian", "-mthumb", "-O2", "-g", "-Wall",
        f"@{generated / 'device.opt'}",
    ]

    if stage:
        flags.append(f"-DCAR_STAGE={stage}")

    objects = []
    source = None
    target = None
    name = ""

    for source in sources:
        name = (source.relative_to(ROOT).as_posix()
                if source.is_relative_to(ROOT) else source.name)
        target = directory / (name.replace("/", "_") + ".o")
        run([COMPILER, *flags, *[f"-I{path}" for path in includes],
             "-c", source, "-o", target], directory)
        objects.append(target)

    driverlib = SDK / (
        "source/ti/driverlib/lib/ticlang/m0p/"
        "mspm0g1x0x_g3x0x/driverlib.a")
    run([COMPILER, *flags, *objects, f"-Wl,-i{generated.as_posix()}",
         "-Wl,-ldevice_linker.cmd", f"-Wl,-l{driverlib.as_posix()}",
         "-Wl,-llibc.a", "-Wl,--rom_model", "-Wl,-m2026_smart_car.map",
         "-o", directory / "2026_smart_car.out"], directory)
    print(f"PASS: stage={stage or 'default'}, "
          f"{len(sources)} sources; {directory}")


def main():
    """Build default configuration, one stage, or all four stages."""
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--stage", type=int, choices=range(1, 5))
    group.add_argument("--all-stages", action="store_true")
    args = parser.parse_args()
    generated = BUILD / "syscfg"
    generated.mkdir(parents=True, exist_ok=True)
    run([SYSCONFIG, "-s", SDK / ".metadata/product.json", "--script",
         ROOT / "empty.syscfg", "-o", generated, "--compiler", "ticlang"],
        generated)
    stage = None

    for stage in (range(1, 5) if args.all_stages else [args.stage]):
        build_stage(stage, generated)


if __name__ == "__main__":
    main()
