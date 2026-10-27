#!/usr/bin/env python3
"""Record or compare deterministic, device-free application invocations."""

import argparse
import copy
import hashlib
import json
from pathlib import Path
import platform
import re
import subprocess


ROOT = Path(__file__).resolve().parents[2]
# Retired configs and protocol fixtures retain the exact frozen bytes.
# Current protocol fixtures remain separate and continue to test current behavior.
LEGACY_RESOURCE_PATHS = {
    "config/camera.yaml": "tests/fixtures/legacy_config/camera.yaml",
    "config/camera.mechanical_5_40.yaml": "tests/fixtures/legacy_config/camera.mechanical_5_40.yaml",
    "tests/fixtures/frame_mapping.synthetic.json": "tests/fixtures/legacy_config/frame_mapping.synthetic.json",
    "config/frame_20261008.demo.json": "tests/fixtures/legacy_config/frame_20261008.demo.json",
    "config/task_calibration.40_candidate.json": "tests/fixtures/legacy_config/task_calibration.40_candidate.json",
    "config/demo_20261007.json": "tests/fixtures/legacy_config/demo_20261007.json",
    "config/demo_20261008.assume.json": "tests/fixtures/legacy_config/demo_20261008.assume.json",
    "config/demo_acceptance.pending.json": "tests/fixtures/legacy_config/demo_acceptance.pending.json",
    "config/demo_acceptance.assume.json": "tests/fixtures/legacy_config/demo_acceptance.assume.json",
    "config/frame_20261007.pending.json": "tests/fixtures/legacy_config/frame_20261007.pending.json",
    "config/frame_20261008.assume.json": "tests/fixtures/legacy_config/frame_20261008.assume.json",
    "config/task_calibration.40_runtime.json": "tests/fixtures/legacy_config/task_calibration.40_runtime.json",
    "config/task_calibration.example.json": "tests/fixtures/legacy_config/task_calibration.example.json",
    "config/zone_color.field.json": "tests/fixtures/legacy_config/zone_color.field.json",
    "config/zone_color.example.json": "tests/fixtures/legacy_config/zone_color.example.json",
    "config/zone_geometry.json": "tests/fixtures/legacy_config/zone_geometry.json",
    "config/rescue.yaml": "tests/fixtures/legacy_config/rescue.yaml",
    "config/runtime/preview.json": "tests/fixtures/legacy_config/runtime/preview.json"
}
CASES = {
    "push_delivery": ["--startup-advance-ms", "0", "--pitch-presets", "500,500,4000",
                      "--push-replay", "tests/fixtures/push_delivery.json"],
    "geometry": ["--team", "red", "--geometry-replay", "tests/fixtures/stage01_geometry.json",
                 "--calibration", "tests/fixtures/stage01-camera.synthetic.json",
                 "--zone-geometry", "tests/fixtures/zone_geometry_inner_v1.json"],
    "check_defaults": ["--check-config"],
    "check_startup": ["--check-config", "--startup-speed", ".1", "--startup-advance-ms", "1000",
                      "--turn-wz", ".2", "--scan-wz", "1.6"],
    "check_mechanical": ["--check-config", "--controlled-empty-field",
                         "--allow-mechanical-pitch-model", "--pitch-presets", "500,4000,4000",
                         "--task-calibration", "config/frame_20261008.assume.json",
                         "--known-frame-angle", "0", "--known-frame-id", "1",
                         "--calibration", "config/camera.mechanical_5_40.yaml",
                         "--zone-color-calibration", "config/zone_color.field.json",
                         "--pose-model", "models/zone_pose_fp.rknn"],
    "check_carry_demo": ["--check-config", "--demo-mode", "carry_once", "--controlled-empty-field",
                         "--assume-all-safe", "--allow-mechanical-pitch-model",
                         "--task-calibration", "config/frame_20261008.assume.json"],
    "invalid_modes": ["--hardware", "--dry-run", "--imu"],
}


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def capture(binary):
    cases = {}
    for name, arguments in CASES.items():
        # Execute relocated fixtures; preserve logical arguments in frozen comparisons.
        executed = [LEGACY_RESOURCE_PATHS.get(value, value) for value in arguments]
        result = subprocess.run([str(binary), *executed], cwd=ROOT,
                                capture_output=True, text=True, timeout=30)
        cases[name] = {"args": arguments, "returncode": result.returncode,
                       "stdout": result.stdout, "stderr": result.stderr}
    return cases


def require_keys(value, keys, name):
    if not isinstance(value, dict) or set(value) != set(keys):
        raise ValueError(f"{name}: expected exactly {', '.join(sorted(keys))}")


def unique_json_object(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"supplement: duplicate key {key!r}")
        value[key] = item
    return value


def load_json(path):
    return json.loads(path.read_text(), object_pairs_hook=unique_json_object)


def validate_provenance(supplement, baseline_path, baseline, name, machine=None):
    require_keys(supplement, ("schema_version", "platform_machine", "baseline_sha256",
                             "legacy_revision", "legacy_binary_sha256", "evidence",
                             "geometry_stdout"), "platform baseline")
    if type(supplement["schema_version"]) is not int or supplement["schema_version"] != 1:
        raise ValueError(f"{name}: unsupported schema_version")
    if supplement["platform_machine"] != "aarch64":
        raise ValueError(f"{name}: only the verified aarch64 platform is supported")
    actual_machine = platform.machine() if machine is None else machine
    if actual_machine != supplement["platform_machine"]:
        raise ValueError(f"{name}: machine {actual_machine!r} does not match aarch64")
    for key, length in (("baseline_sha256", 64), ("legacy_revision", 40),
                        ("legacy_binary_sha256", 64)):
        value = supplement[key]
        if not isinstance(value, str) or not re.fullmatch(f"[0-9a-f]{{{length}}}", value):
            raise ValueError(f"{name}: invalid {key}")
    if digest(baseline_path) != supplement["baseline_sha256"]:
        raise ValueError(f"{name}: frozen baseline SHA-256 differs")
    if baseline.get("revision") != supplement["legacy_revision"]:
        raise ValueError(f"{name}: legacy revision differs")
    evidence = supplement["evidence"]
    if not isinstance(evidence, list) or not evidence or any(not isinstance(item, str) or not item for item in evidence):
        raise ValueError(f"{name}: evidence must be a nonempty list of references")


def apply_platform_baseline(baseline, path, baseline_path, machine=None):
    supplement = load_json(path)
    validate_provenance(supplement, baseline_path, baseline, "platform baseline", machine=machine)
    replacement = supplement["geometry_stdout"]
    require_keys(replacement, ("from", "to"), "platform baseline geometry_stdout")
    line_pattern = r"frame=1 zone_valid=1 points=6 residual_m=[0-9]+(?:\.[0-9]+)?(?:e[+-]?[0-9]+)? reason=ok subzone=supply\n"
    for name in ("from", "to"):
        if not isinstance(replacement[name], str) or not re.fullmatch(line_pattern, replacement[name]):
            raise ValueError(f"platform baseline: {name} must be the complete geometry frame 1 residual line")
    if replacement["from"] == replacement["to"]:
        raise ValueError("platform baseline: geometry residual replacement must differ")
    stdout = baseline["cases"]["geometry"]["stdout"]
    if stdout.count(replacement["from"]) != 1:
        raise ValueError("platform baseline: geometry source line must occur exactly once")
    expected = copy.deepcopy(baseline)
    expected["cases"]["geometry"]["stdout"] = stdout.replace(replacement["from"], replacement["to"], 1)
    return expected


def apply_protocol_baseline(baseline, path, baseline_path):
    """Apply only the exact, counted protocol-output changes declared in a versioned supplement."""
    supplement = load_json(path)
    require_keys(supplement, ("schema_version", "protocol", "baseline_sha256",
                              "legacy_revision", "evidence", "replacements"),
                 "protocol baseline")
    if type(supplement["schema_version"]) is not int or supplement["schema_version"] != 1:
        raise ValueError("protocol baseline: unsupported schema_version")
    protocol_name = supplement["protocol"]
    if not isinstance(protocol_name, str) or not re.fullmatch(r"[A-Za-z0-9_.-]+", protocol_name):
        raise ValueError("protocol baseline: protocol must be a nonempty identifier")
    for key, length in (("baseline_sha256", 64), ("legacy_revision", 40)):
        value = supplement[key]
        if not isinstance(value, str) or not re.fullmatch(f"[0-9a-f]{{{length}}}", value):
            raise ValueError(f"protocol baseline: invalid {key}")
    if digest(baseline_path) != supplement["baseline_sha256"]:
        raise ValueError("protocol baseline: frozen baseline SHA-256 differs")
    if baseline.get("revision") != supplement["legacy_revision"]:
        raise ValueError("protocol baseline: legacy revision differs")
    evidence = supplement["evidence"]
    if not isinstance(evidence, list) or not evidence or any(not isinstance(item, str) or not item for item in evidence):
        raise ValueError("protocol baseline: evidence must be a nonempty list of references")
    replacements = supplement["replacements"]
    if not isinstance(replacements, list) or not replacements:
        raise ValueError("protocol baseline: replacements must be a nonempty list")

    expected = copy.deepcopy(baseline)
    for index, replacement in enumerate(replacements):
        name = f"protocol baseline replacement {index}"
        require_keys(replacement, ("case", "field", "from", "to", "count"), name)
        case = replacement["case"]
        if case not in expected["cases"]:
            raise ValueError(f"{name}: unknown case {case!r}")
        if replacement["field"] != "stdout":
            raise ValueError(f"{name}: only stdout may be reconciled")
        for key in ("from", "to"):
            value = replacement[key]
            if not isinstance(value, str) or not value or "\x00" in value:
                raise ValueError(f"{name}: {key} must be a nonempty string")
        if replacement["from"] == replacement["to"]:
            raise ValueError(f"{name}: replacement must differ")
        if replacement["from"] in replacement["to"]:
            raise ValueError(f"{name}: to must not contain from")
        count = replacement["count"]
        if type(count) is not int or count <= 0:
            raise ValueError(f"{name}: count must be a positive integer")
        stdout = expected["cases"][case]["stdout"]
        if stdout.count(replacement["from"]) != count:
            raise ValueError(f"{name}: source text must occur exactly {count} times")
        expected["cases"][case]["stdout"] = stdout.replace(replacement["from"], replacement["to"])
    return expected


def load_baseline(path, platform_baseline=None, machine=None, protocol_baseline=None):
    baseline = json.loads(path.read_text())
    expected = copy.deepcopy(baseline)
    if protocol_baseline is not None:
        expected = apply_protocol_baseline(expected, protocol_baseline, path)
    if platform_baseline is not None:
        expected = apply_platform_baseline(expected, platform_baseline, path, machine=machine)
    return expected


def comparison_failures(cases, baseline, root=ROOT):
    failures = []
    for relative, expected_hash in baseline.get("resources_sha256", {}).items():
        resource = root / LEGACY_RESOURCE_PATHS.get(relative, relative)
        if not resource.is_file() or digest(resource) != expected_hash:
            failures.append(f"{relative}: baseline resource differs")
    for name, actual in cases.items():
        expected = baseline["cases"].get(name)
        if expected is None:
            failures.append(f"{name}: baseline missing")
            continue
        for key in ("args", "returncode", "stdout", "stderr"):
            if actual[key] != expected[key]:
                failures.append(f"{name}: {key} differs")
    return failures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("--record", type=Path)
    parser.add_argument("--compare", type=Path)
    parser.add_argument("--protocol-baseline", type=Path,
                        help="versioned protocol-output supplement for --compare")
    parser.add_argument("--platform-baseline", type=Path,
                        help="explicit verified platform supplement for --compare")
    args = parser.parse_args()
    if bool(args.record) == bool(args.compare):
        parser.error("choose exactly one of --record and --compare")
    if args.protocol_baseline and not args.compare:
        parser.error("--protocol-baseline is only allowed with --compare")
    if args.platform_baseline and not args.compare:
        parser.error("--platform-baseline is only allowed with --compare")
    binary = args.binary.resolve(strict=True)
    cases = capture(binary)
    if args.record:
        revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
        status = subprocess.check_output(["git", "status", "--short"], cwd=ROOT, text=True)
        resources = {str(path.relative_to(ROOT)): digest(path)
                     for directory in (ROOT / "config", ROOT / "tests/fixtures")
                     for path in sorted(directory.rglob("*")) if path.is_file()}
        record = {"schema_version": 1, "evidence": "synthetic_offline",
                  "revision": revision, "worktree_status": status,
                  "binary": str(binary), "binary_sha256": digest(binary),
                  "cwd": str(ROOT), "resources_sha256": resources, "cases": cases}
        args.record.parent.mkdir(parents=True, exist_ok=True)
        args.record.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n")
        print(f"Recorded {len(cases)} offline cases: {args.record.resolve()}")
        return 0
    try:
        baseline = load_baseline(args.compare, args.platform_baseline,
                                 protocol_baseline=args.protocol_baseline)
    except ValueError as error:
        parser.error(str(error))
    failures = comparison_failures(cases, baseline)
    if failures:
        print("\n".join(failures))
        return 1
    supplements = [name for name, path in (("protocol", args.protocol_baseline),
                                            ("platform", args.platform_baseline)) if path]
    if supplements:
        print(f"All {len(cases)} offline cases match the baseline with the explicit "
              f"{' and '.join(supplements)} supplement{'s' if len(supplements) > 1 else ''} exactly")
    else:
        print(f"All {len(cases)} offline cases match the baseline exactly")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
