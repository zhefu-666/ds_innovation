#!/usr/bin/env python3
"""Check or fix project C/C++ formatting with clang-format 18.1.8."""

import argparse
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
VERSION = "18.1.8"
EXTENSIONS = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"}


def source_files():
    output = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
        cwd=ROOT, check=True, stdout=subprocess.PIPE,
    ).stdout
    files = []
    for name in sorted(set(output.decode().split("\0")) - {""}):
        path = Path(name)
        owned = path.parts[0] in {"src", "include", "tests"} or path.parts[:2] == (
            "foxglove_telemetry", "src"
        )
        if owned and path.suffix in EXTENSIONS and (ROOT / path).is_file():
            files.append(path)
    return files


def formatter():
    bundled = ROOT / ".tools" / "venv" / "bin" / "clang-format"
    executable = str(bundled) if bundled.is_file() else shutil.which("clang-format")
    if not executable:
        raise RuntimeError(
            "clang-format 18.1.8 not found; install the pinned tool as described in docs/FORMATTING.md"
        )
    result = subprocess.run(
        [executable, "--version"], check=True, capture_output=True, text=True
    )
    version = re.search(r"clang-format version (\d+\.\d+\.\d+)", result.stdout)
    if not version or version.group(1) != VERSION:
        raise RuntimeError(f"Expected clang-format {VERSION}; found {result.stdout.strip()}")
    return executable


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--check", action="store_true", help="report differences without writing")
    action.add_argument("--fix", action="store_true", help="rewrite only files with formatting differences")
    args = parser.parse_args()
    if not (ROOT / ".clang-format").is_file():
        raise RuntimeError(f"Missing formatting rules: {ROOT / '.clang-format'}")
    executable = formatter()
    files = source_files()
    if not files:
        raise RuntimeError("No project C/C++ files found")
    changed = 0
    for relative in files:
        path = ROOT / relative
        original = path.read_bytes()
        formatted = subprocess.run(
            [executable, "--style=file", "--fallback-style=none",
             "--assume-filename=" + str(path)],
            input=original, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True,
        ).stdout
        if formatted == original:
            continue
        changed += 1
        if args.fix:
            if path.read_bytes() != original:
                raise RuntimeError(f"File changed during formatting: {relative}; rerun on current content")
            path.write_bytes(formatted)
        print(f"{'FIXED' if args.fix else 'NEEDS FORMAT'} {relative}")
    print(f"clang-format {VERSION}: checked {len(files)} files; "
          f"{changed} {'updated' if args.fix else 'need formatting'}")
    return 1 if args.check and changed else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"format: {error}", file=sys.stderr)
        if isinstance(error, subprocess.CalledProcessError) and error.stderr:
            detail = error.stderr.decode(errors="replace") if isinstance(error.stderr, bytes) else error.stderr
            print(detail.rstrip(), file=sys.stderr)
        sys.exit(2)
