#!/usr/bin/env python3
"""把 colcon 每个包生成的 compile_commands.json 合并成一份，供 VS Code IntelliSense 使用。

用法：
    ./scripts/merge_compile_commands.py
（每次 colcon build 之后跑一次；也可以在 CMake 里加 custom target 自动跑）
"""
from __future__ import annotations

import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
BUILD_DIR = ROOT / "build"


def main() -> int:
    if not BUILD_DIR.is_dir():
        print(f"找不到 {BUILD_DIR}，先执行 colcon build", file=sys.stderr)
        return 1

    entries: list[dict] = []
    files = sorted(BUILD_DIR.glob("*/compile_commands.json"))
    for path in files:
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            print(f"跳过 {path}: {exc}", file=sys.stderr)
            continue
        entries.extend(data)

    out = BUILD_DIR / "compile_commands.json"
    out.write_text(json.dumps(entries, indent=2), encoding="utf-8")
    print(f"合并 {len(files)} 个包，共 {len(entries)} 条编译命令 -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
