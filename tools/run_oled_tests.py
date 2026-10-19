"""Run real menu/canvas/OLED C sources against I2C mocks; export previews."""

from pathlib import Path
import html
import shutil
import struct
import subprocess
import zlib


def write_png(source):
    """Convert the real 128x64 page-layout framebuffer into a 4x preview."""
    frame = source.read_bytes()
    rows = []
    row = bytearray()
    x = 0
    y = 0
    value = 0

    for y in range(256):
        row = bytearray([0])

        for x in range(512):
            value = frame[(y // 4 // 8) * 128 + x // 4]
            row.append(255 if value & (1 << (y // 4 % 8)) else 0)

        rows.append(bytes(row))

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(
            ">I", zlib.crc32(kind + data) & 0xFFFFFFFF)

    image = b"\x89PNG\r\n\x1a\n"
    image += chunk(b"IHDR", struct.pack(">IIBBBBB", 512, 256, 8, 0, 0, 0, 0))
    image += chunk(b"IDAT", zlib.compress(b"".join(rows)))
    image += chunk(b"IEND", b"")
    source.with_suffix(".png").write_bytes(image)


def write_animation(timeline):
    """Export physical frame completions as a dependency-free monochrome GIF."""
    entries = [line.rsplit(",", 1) for line in timeline.read_text().splitlines()]
    output = bytearray(b"GIF89a" + struct.pack("<HHBBB", 256, 128, 128, 0, 0))
    output += b"\x00\x00\x00\xff\xff\xff"
    output += b"\x21\xff\x0bNETSCAPE2.0\x03\x01\x00\x00\x00"
    index = 0
    name = ""
    timestamp = ""
    frame = b""
    delay = 0
    payload = bytearray()
    accumulator = 0
    bits = 0
    x = 0
    y = 0
    value = 0
    offset = 0

    for index, (name, timestamp) in enumerate(entries):
        frame = (timeline.parent / name).read_bytes()
        delay = max(2, round((int(entries[index + 1][1]) - int(timestamp)) / 10)
                    if index + 1 < len(entries) else 8)
        output += b"\x21\xf9\x04\x04" + struct.pack("<HBB", delay, 0, 0)
        output += b"\x2c" + struct.pack("<HHHHB", 0, 0, 256, 128, 0)
        payload = bytearray()
        accumulator = 0
        bits = 0

        for y in range(128):
            for x in range(256):
                value = (frame[(y // 2 // 8) * 128 + x // 2] >>
                         (y // 2 % 8)) & 1
                # Clear + literal keeps the LZW code width fixed at 3 bits.
                accumulator |= (4 | (value << 3)) << bits
                bits += 6

                while bits >= 8:
                    payload.append(accumulator & 255)
                    accumulator >>= 8
                    bits -= 8

        accumulator |= 5 << bits
        bits += 3

        while bits > 0:
            payload.append(accumulator & 255)
            accumulator >>= 8
            bits -= 8

        output.append(2)

        for offset in range(0, len(payload), 255):
            output.append(len(payload[offset:offset + 255]))
            output += payload[offset:offset + 255]

        output.append(0)

    output.append(0x3B)
    timeline.with_suffix(".gif").write_bytes(output)


def write_animation_index(build):
    """Make an offline page for inspecting the exported transition sequences."""
    sequences = sorted(build.glob("menu_anim_*.gif"))
    links = "".join(
        f'<button onclick="preview.src=\'{html.escape(path.name)}\'">'
        f'{html.escape(path.stem.removeprefix("menu_anim_"))}</button>'
        for path in sequences)
    page = f"""<!doctype html><meta charset="utf-8">
<title>Smart car OLED animation</title>
<style>body{{background:#151719;color:#eee;font:16px sans-serif;
max-width:800px;margin:40px auto;padding:16px}}button{{margin:5px;padding:8px}}
img{{display:block;width:512px;max-width:100%;image-rendering:pixelated;
margin:24px 0;border:16px solid #050505;box-sizing:border-box}}</style>
<h1>OLED 动画预览</h1><p>KEY1 上 · KEY2 下 · KEY3 确认 · KEY4 返回/停止</p>
<div>{links}</div><img id="preview" src="menu_anim_open_list.gif">
<p>帧来自真实 C 菜单和刷新驱动；按 400 kHz 估算线时，实际流畅度待上板测量。</p>
"""
    (build / "menu_animation.html").write_text(page, encoding="utf-8")


def main():
    """Compile and run host display tests without accessing real hardware."""
    root = Path(__file__).resolve().parents[1]
    build = root / "Debug/oled_tests"
    build.mkdir(parents=True, exist_ok=True)
    compiler = shutil.which("gcc")

    if compiler is None:
        raise SystemExit("gcc was not found on PATH")

    includes = [
        "tests/mock", "application", "application/menu", "application/race",
        "modules/vision", "modules/slope", "modules/oled", "bsp/i2c",
        "bsp/delay",
    ]
    sources = [
        "tests/oled_menu_test.c", "modules/oled/oled.c",
        "modules/oled/oled_canvas.c", "application/menu/car_menu.c",
        "application/menu/car_menu_data.c", "application/tuning/car_tuning.c",
    ]
    executable = build / "oled_menu_test.exe"
    command = [
        compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-Wno-unused-parameter", "-fstack-protector-all",
    ]
    command += [f"-I{root / item}" for item in includes]
    command += [str(root / item) for item in sources]
    command += ["-lm", "-o", str(executable)]
    subprocess.run(command, cwd=root, check=True)
    subprocess.run([str(executable)], cwd=build, check=True, timeout=10)
    source = None

    for source in build.glob("menu_*.bin"):
        write_png(source)

    for source in build.glob("menu_anim_*.csv"):
        write_animation(source)

    write_animation_index(build)
    print(f"OLED previews: {build}")


if __name__ == "__main__":
    main()
