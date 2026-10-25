#!/usr/bin/env python3
"""Exercise configuration export through the real application without devices."""

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile


class Failure(RuntimeError):
    pass


def require(condition, message):
    if not condition:
        raise Failure(message)


def strict_json(text):
    def unique_object(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise Failure(f"Export contains duplicate key: {key}")
            result[key] = value
        return result

    def invalid_constant(value):
        raise Failure(f"Export contains a non-JSON numeric constant: {value}")

    try:
        result = json.loads(text, object_pairs_hook=unique_object,
                            parse_constant=invalid_constant)
    except json.JSONDecodeError as error:
        raise Failure(f"Expected one complete JSON document: {error}") from error
    require(isinstance(result, dict), "Export root must be an object")
    return result


def run(binary, arguments, cwd, expected=0, error_fragment=None, preexec_fn=None):
    command = [str(binary), *map(str, arguments)]
    try:
        result = subprocess.run(command, cwd=cwd, capture_output=True,
                                text=True, timeout=10, check=False,
                                preexec_fn=preexec_fn)
    except subprocess.TimeoutExpired as error:
        raise Failure(f"Configuration invocation did not terminate: {command}") from error
    detail = (f"Command: {command}\nExit: {result.returncode}\n"
              f"stdout: {result.stdout[:4000]}\nstderr: {result.stderr[:4000]}")
    require(result.returncode == expected, detail)
    if error_fragment is not None:
        require(error_fragment in result.stderr, detail)
    return result


def write_profile(path, data):
    path.write_text(json.dumps(data, ensure_ascii=True, indent=2) + "\n",
                    encoding="utf-8")


def source_history(document, field):
    entries = [entry["history"] for entry in document["field_sources"]
               if entry["field"] == field]
    require(len(entries) == 1, f"Expected one provenance entry for {field}")
    return entries[0]


def effective_values(document):
    # The invocation arguments intentionally differ when changing flag order or
    # export destination; the resolved configuration must remain identical.
    identity = {key: value for key, value in document.get("identity", {}).items()
                if key != "arguments"}
    return {**document, "identity": identity}


def verify(binary, repository, directory):
    work = directory / "working-directory"
    profiles = directory / "profiles"
    work.mkdir()
    profiles.mkdir()
    # Missing inputs make an accidental transition into runtime fail before any
    # camera, NPU, real serial port or match socket can be initialized.
    absent_imu = directory / "never-open-imu"
    absent_mcu = directory / "never-open-mcu"
    socket = directory / "never-create-match.sock"
    absent_model = directory / 'missing "model".rknn'
    safe_live = ["--dry-run", "--imu", "--imu-port", absent_imu,
                 "--port", absent_mcu, "--model", absent_model,
                 "--rknn-library", directory / "missing-rknn.so",
                 "--match-socket", socket, "--camera", "255", "--no-show"]

    result = run(binary, [*safe_live, "--startup-advance-ms", "5000",
                         "--startup-speed", ".25", "--turn-wz", ".9",
                         "--dump-config", "-"], work)
    dumped = strict_json(result.stdout)
    require(dumped["schema_version"] == 1, "Unexpected export schema")
    require(dumped["policy"]["name"] == "legacy_field_v1", "Wrong field policy")
    require(dumped["request"]["model_path"] == str(absent_model), "CLI path was changed")
    require(dumped["startup"]["requested_advance_ms"] == 5000, "Lost startup request")
    require(dumped["startup"]["effective_advance_us"] == 1250000,
            "Export did not include the resolved startup duration")
    require(dumped["startup"]["effective_speed_mps"] == 1,
            "Export did not include the resolved startup speed")
    require(math.isclose(dumped["task"]["turn_wz"], .55, abs_tol=1e-6),
            "Task export incorrectly reports the requested turn rate")
    require(dumped["capture"]["mouth_y_m"] == "+infinity",
            "Uncalibrated capture default must have a legal JSON encoding")
    require("MATCH_TIME_LIMIT=" in result.stderr,
            "Startup diagnostics were lost instead of redirected from stdout")
    require(not socket.exists(), "Dump created the runtime control socket")

    library_search = strict_json(run(binary, ["--rknn-library", "librknnrt.so",
                                              "--dump-config", "-"], work).stdout)
    require(library_search["request"]["rknn_library"] == "librknnrt.so",
            "Bare RKNN library request was rewritten")
    require(library_search["identity"]["rknn_library_lookup"] == "dynamic_linker_search",
            "Bare RKNN library name was not identified as a dynamic-linker search")
    require("rknn_library" not in library_search["identity"]["resource_paths"],
            "Audit fabricated a working-directory path for a dynamic-linker library name")

    if sys.platform.startswith("linux"):
        fixture = repository / "tests/fixtures/push_delivery.json"
        require(fixture.is_file(), "Missing absolute replay fixture")
        replay_arguments = ["--startup-advance-ms", "0", "--pitch-presets", "500,500,4000",
                            "--push-replay", fixture]

        def in_deleted_working_directory(arguments, name):
            removed_directory = str(directory / name)
            os.mkdir(removed_directory)

            def remove_cwd():
                os.chdir(removed_directory)
                os.rmdir(removed_directory)

            return run(binary, arguments, work, preexec_fn=remove_cwd)

        replay_result = in_deleted_working_directory(replay_arguments, "deleted-replay-cwd")
        require("state=SCAN batch=0 delivered=1 first=1" in replay_result.stdout,
                "Absolute replay failed to complete delivery after its working directory was deleted")
        deleted_dump = strict_json(in_deleted_working_directory(
            [*replay_arguments, "--dump-config", "-"], "deleted-dump-cwd").stdout)
        require(bool(deleted_dump["identity"]["audit_error"]),
                "Deleted working directory was not recorded as an audit limitation")
        require(deleted_dump["identity"]["working_directory"] == "",
                "Audit reported a working directory after it ceased to exist")
        require(deleted_dump["request"]["push_replay"] == str(fixture) and
                deleted_dump["identity"]["resource_paths"]["push_replay"] == str(fixture),
                "Deleted working directory lost the usable absolute replay reference")
        resources = deleted_dump["identity"]["resource_paths"]
        for name in ("model_path", "pose_model_path", "rknn_library", "calibration_file",
                     "task_calibration_file", "zone_geometry_file", "zone_color_file",
                     "keypoints_file", "geometry_replay", "detect_image"):
            reference = deleted_dump["request"][name]
            if reference and not Path(reference).is_absolute():
                require(name not in resources,
                        f"Audit fabricated an absolute {name} after its working directory was deleted")

    profile = profiles / "site.json"
    config = {
        "schema_version": 1,
        "behavior_policy": "legacy_field_v1",
        "hardware": {"fps": 17, "camera_index": 255, "imu": True,
                     "imu_port": "devices/never-open-imu",
                     "uart_port": "devices/never-open-mcu"},
        "models": {"model_path": "../assets/missing detector.rknn",
                   "rknn_library": "../assets/missing-runtime.so"},
        "behavior": {"startup_advance_ms": 5000, "startup_speed_mps": .25,
                     "turn_wz_rps": .2},
        "runtime": {"dry_run": True, "show": False,
                    "match_socket": "never-create.sock", "vision_log_dir": "none"},
    }
    write_profile(profile, config)
    result = run(binary, ["--config", "../profiles/site.json", "--dump-config", "-"], work)
    from_file = strict_json(result.stdout)
    require(from_file["request"]["model_path"] ==
            str((profiles / config["models"]["model_path"]).resolve()),
            "File model path was not resolved relative to its config")
    require(from_file["request"]["imu_port"] == str(profiles / "devices/never-open-imu"),
            "File device reference was resolved against the working directory")
    require(from_file["request"]["fps"] == 17, "CLI defaults overwrote the file")
    require(from_file["request"]["vision_log_dir"] == "none", "Logging sentinel became a path")
    require(from_file["identity"]["config_file"] == str(profile), "Configuration identity lost its absolute path")
    require(from_file["identity"]["working_directory"] == str(work), "Wrong working-directory identity")
    require(from_file["identity"]["executable"] == str(binary), "Wrong executable identity")
    require(not (profiles / "never-create.sock").exists(), "File dump created a runtime socket")

    cli_model = "cli-assets/missing detector.rknn"
    overrides = ["--fps", "29", "--model", cli_model, "--startup-speed", ".5",
                 "--startup-advance-ms", "4000", "--turn-wz", ".9"]
    expected_override = None
    for arguments in ([*overrides, "--config", "../profiles/site.json"],
                      ["--config", "../profiles/site.json", *overrides]):
        result = run(binary, [*arguments, "--dump-config", "-"], work)
        actual = strict_json(result.stdout)
        require(actual["request"]["fps"] == 29, "Explicit FPS did not override file")
        require(actual["request"]["model_path"] == cli_model,
                "CLI path must retain working-directory interpretation")
        require(actual["request"]["rknn_library"] ==
                str((profiles / config["models"]["rknn_library"]).resolve()),
                "Unrelated file path changed while applying a CLI override")
        require(actual["startup"]["effective_advance_us"] == 2000000,
                "Final startup configuration did not use the CLI request")
        history = source_history(actual, "request.model_path")
        require(f"config:{profile}#models.model_path" in history,
                "File provenance was omitted")
        require(history[-1] == "explicit_cli", "Explicit CLI provenance did not follow the file")
        if expected_override is None:
            expected_override = actual
        else:
            require(effective_values(actual) == effective_values(expected_override),
                    "Position of --config changed resolution")

    output = work / "effective.json"
    result = run(binary, ["--config", "../profiles/site.json", *overrides,
                         "--dump-config", "effective.json"], work)
    require(output.is_file(), "Relative export destination was not written in the working directory")
    require(effective_values(strict_json(output.read_text(encoding="utf-8"))) ==
            effective_values(expected_override),
            "File export differs from stdout export")
    require(not result.stdout.lstrip().startswith("{"), "File export also emitted JSON on stdout")

    demo_profile = profiles / "recognize.json"
    write_profile(demo_profile, {
        "schema_version": 1, "behavior_policy": "legacy_field_v1",
        "hardware": {"camera_index": 255},
        "models": {"model_path": "missing-detector.rknn",
                   "rknn_library": "missing-runtime.so",
                   "pose_model_path": "generic-zone.rknn",
                   "pose_model_blue_path": "blue-zone.rknn",
                   "pose_model_red_path": "red-zone.rknn"},
        "behavior": {"startup_advance_ms": 5000, "startup_speed_mps": .4,
                     "scan_wz_rps": 1.2, "turn_wz_rps": .9},
        "runtime": {"demo_mode": "recognize", "demo_seconds": 17,
                    "show": False, "match_socket": "demo-never-create.sock"},
    })
    for team in ("red", "blue"):
        demo = strict_json(run(binary, ["--config", demo_profile, "--team", team,
                                       "--pose-model", "cli-generic-zone.rknn",
                                       "--startup-advance-ms", "3000", "--startup-speed", ".6",
                                       "--scan-wz", "1.4", "--turn-wz", "1.1",
                                       "--dump-config", "-"], work).stdout)
        request = demo["request"]
        require(request["startup_advance_ms"] == 0 and
                math.isclose(request["startup_speed_mps"], .1, abs_tol=1e-6),
                "Recognition normalization did not replace explicit startup requests")
        require(math.isclose(request["turn_wz_rps"], .25, abs_tol=1e-6),
                "Recognition normalization did not bound the requested turn rate")
        require(request["dry_run"] == 1 and request["match_seconds"] == 17,
                "Recognition normalization omitted its output or duration limits")
        for field, ending in (
            ("startup_advance_ms", "sets startup_advance_ms=0"),
            ("startup_speed_mps", "sets startup_speed_mps=0.10"),
            ("turn_wz_rps", "bounds turn_wz_rps to 0.25"),
        ):
            history = source_history(demo, "request." + field)
            require(f"config:{demo_profile}#behavior.{field}" in history and
                    "explicit_cli" in history,
                    f"Normalization discarded the input provenance for {field}")
            require(history[-1] == f"parseArgs: demo_mode=recognize {ending}",
                    f"Final source of {field} incorrectly claims the CLI supplied its normalized value")
        selected_pose = str(profiles / f"{team}-zone.rknn")
        require(request["pose_model_path"] == selected_pose,
                "Team-specific pose selection did not replace the explicit generic model")
        pose_history = source_history(demo, "request.pose_model_path")
        require("explicit_cli" in pose_history and
                pose_history[-1] == f"parseArgs: team={team} selects pose_model_{team}_path",
                "Final model source omitted team-specific normalization")
        require(demo["identity"]["resource_paths"]["pose_model_path"] == selected_pose,
                "Effective model identity points to the overridden generic model")
        require(demo["task"]["startup_advance_us"] == 0 and
                math.isclose(demo["task"]["turn_wz"], .55, abs_tol=1e-6),
                "Demo request normalization was confused with final legacy task tuning")
        require(not (profiles / "demo-never-create.sock").exists(),
                "Recognition dump initialized the runtime socket")

    replay_profile = profiles / "replay.json"
    write_profile(replay_profile, {
        "schema_version": 1, "behavior_policy": "legacy_replay_v1",
        "runtime": {"push_replay": "missing-recording.json"},
    })
    replay = strict_json(run(binary, ["--config", replay_profile,
                                      "--dump-config", "-"], work).stdout)
    require(replay["policy"]["name"] == "legacy_replay_v1", "Replay policy was lost")
    require(replay["task"]["field_carry"] == 0, "Replay export applied field transport overrides")
    require(replay["request"]["push_replay"] == str(profiles / "missing-recording.json"),
            "Replay reference was not resolved relative to its config")

    # These live requests intentionally name only nonexistent temporary ports.
    # The expected parser error must win over hardware readiness and model errors.
    invalid_live = ["--hardware", "--imu", "--imu-port", absent_imu,
                    "--port", absent_mcu, "--model", absent_model]
    bad = profiles / "invalid.json"
    for text, message in [
        ('{"schema_version":1,"behavior_policy":"legacy_field_v1",', "Runtime configuration"),
        ('{"schema_version":1,"schema_version":1,"behavior_policy":"legacy_field_v1"}', "Duplicate JSON key"),
        (json.dumps({"schema_version": 1, "behavior_policy": "strict_candidate_v1"}), "behavior_policy"),
        (json.dumps({"schema_version": 1, "behavior_policy": "legacy_replay_v1"}), "does not match the selected operation"),
    ]:
        bad.write_text(text, encoding="utf-8")
        result = run(binary, ["--config", bad, *invalid_live,
                              "--dump-config", "-"], work, expected=1,
                     error_fragment=message)
        require(not result.stdout, "Invalid input emitted a partial JSON export")
        require(not socket.exists(), "Invalid input reached runtime socket initialization")

    missing_parent = work / "not-created" / "effective.json"
    for destination in (missing_parent, profiles):
        run(binary, [*safe_live, "--dump-config", destination], work, expected=1,
            error_fragment="Cannot open effective configuration output")
    require(not missing_parent.exists(), "Failed output unexpectedly created a file")
    require(not missing_parent.parent.exists(), "Export silently created a missing output directory")

    checked = run(binary, ["--check-config", "--zone-geometry",
                           repository / "config/zone_geometry.json",
                           "--dump-config", "-"], work, expected=2)
    require(strict_json(checked.stdout)["request"]["check_config"] == 1,
            "Combined dump/check omitted the static-check request")
    require("STATIC_ONLY:" in checked.stderr,
            "Combined dump/check did not keep static-check diagnostics off stdout")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("repository", type=Path)
    arguments = parser.parse_args()
    binary = arguments.binary.resolve(strict=True)
    repository = arguments.repository.resolve(strict=True)
    try:
        with tempfile.TemporaryDirectory(prefix="rescue-config-integration-") as temporary:
            verify(binary, repository, Path(temporary))
    except (Failure, OSError, KeyError, TypeError) as error:
        print(f"Runtime configuration integration failed: {error}", file=sys.stderr)
        return 1
    print("Application configuration dump, precedence, paths and static failures passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
