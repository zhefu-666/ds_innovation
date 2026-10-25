# Runtime configuration schema v1

The application accepts one strict JSON runtime configuration:

```sh
./build/rescue_upper_host --config config/runtime/preview.json
./build/rescue_upper_host --config config/runtime/preview.json --fps 30
./build/rescue_upper_host --config config/runtime/preview.json --dump-config /tmp/effective_config.json
./build/rescue_upper_host --config config/runtime/preview.json --dump-config -
```

`--config` and `--dump-config` each accept one file path and may appear only once.
Use `--dump-config -` for pure JSON on stdout, with diagnostics on stderr.
`--dump-config` requests an export from the application after legacy resolution;
`-` selects standard output instead of a file.
the loader itself does not write files. It can also be used without `--config`.
The preview example disables MCU output and image logging; it still loads the
detector and opens the selected camera when run normally.

## Request precedence and compatibility

Request parameters are resolved in this order:

```text
Config defaults -> one JSON file -> explicitly supplied CLI arguments
              -> existing parseArgs validation and derived values
              -> explicitly named legacy runtime policy
```

The loader converts typed file fields into existing CLI arguments, prepends
them to explicit CLI arguments, and calls `parseArgs` once. CLI defaults cannot
overwrite a field from the file. CLI argument order relative to `--config`
does not change that precedence. Existing mode dependencies, selected-team pose
model selection, demo limits and safety checks remain in `parseArgs`.

The two legacy policies preserve the current final tuning order. In particular,
field tuning and calibration may still override requested behavior parameters
after parsing. This release unifies request sources; it does not introduce a new
final tuning precedence. Inspect both `request` and final values in the exported
configuration before considering a parameter effective. The next configuration
semantics change must be verified separately from this compatibility extraction.

The export also records operation, requested MCU access, task scope, absolute
configuration/executable/resource paths, working directory, arguments and compiled
backend capabilities. These describe the configured run; a dump or static check
does not open the requested hardware. File contents and field acceptance are not
verified by absolute paths alone. The baseline recorder separately hashes its
binary and resources. Existing unused `Config` fields remain visible for audit;
their presence in the snapshot does not make them active schema inputs.
The resolved snapshot is an audit document, not an input runtime profile; retain
the original JSON profile and explicit arguments for reproduction. If the working
directory is unavailable, identity auditing records the error and omits unresolved
relative references; absolute-path offline operations remain usable.
Bare runtime-library names such as `librknnrt.so` retain `dlopen` search semantics:
`identity.rknn_library_lookup` reports `dynamic_linker_search`, and `resource_paths`
does not invent a working-directory location for that name. Explicit library paths
are normalized as path references; a dump does not load or locate the shared library.

Without `--config`, existing CLI parsing and path interpretation are preserved.
File references are resolved relative to the directory containing the config
file and normalized to absolute paths. CLI paths retain their existing working
directory semantics. Empty optional paths remain empty, and `vision_log_dir`
value `"none"` remains the logging-disable sentinel. Paths do not expand `~`
or environment variables. The loader opens only the JSON file; input assets,
models, calibration contents and device readiness are checked by their existing
owners, including the application's `--check-config` path.

For each file field the loader records a source such as
`config:/absolute/runtime.json#models.model_path`, including explicit `false`
values. Explicit CLI fields are tracked separately. The loader also records the
later `parseArgs` transformations for demo bounds, assumed clearance and
selected-team pose models. The runtime resolver records subsequent legacy
tuning overrides.

## Schema

The root must be an object. These two fields are required:

| Field | Type | Supported values |
| --- | --- | --- |
| `schema_version` | Integer | `1` |
| `behavior_policy` | String | `legacy_field_v1`, `legacy_replay_v1` |

`legacy_replay_v1` requires a nonempty `push_replay` request.
`legacy_field_v1` applies to other operations, including live preview, demos,
single-image inference and geometry replay. The policy is checked after explicit
CLI overrides. A CLI operation that conflicts with the file policy fails.
These policy names describe the existing tuning branches, not a grant of
hardware access or a statement that measured safety evidence is available.

The optional groups below are objects. All keys must be spelled exactly as
listed. Unknown keys, duplicate keys at any level, nulls, wrong types, invalid
numbers, comments and trailing non-JSON input are rejected. A numeric `0`/`1`
does not stand in for a boolean. Integer fields do not accept `30.0`. Omitted
fields retain the defaults in `Config`.

All positive boolean switches currently default to `false`; `runtime.show`
defaults to `true`. File `false` therefore takes effect without adding a
positive switch, and `show: false` becomes `--no-show`. Explicit CLI switches
are appended afterward. Existing CLI switches are one-way: for example,
`--telemetry` can enable file `telemetry: false`, while disabling file
`telemetry: true` requires changing that file. There are no implicit negative
CLI flags or environment overlays.

### Hardware

| Key | Type | Constraints / equivalent CLI |
| --- | --- | --- |
| `uart_port` | Path string | `--port` |
| `baudrate` | Integer | 1..4000000; `--baud` |
| `camera_index` | Integer | 0..255; `--camera` |
| `frame_width`, `frame_height` | Integer | 1..16384; `--width`, `--height` |
| `fps` | Integer | 1..1000; `--fps` |
| `imu` | Boolean | `--imu` |
| `imu_port` | Path string | `--imu-port` |
| `imu_baud` | Integer | Existing supported rates: 9600, 115200, 230400, 460800, 921600 |
| `imu_timeout_ms` | Integer | 1..10000; `--imu-timeout-ms` |
| `pitch_feedback` | Boolean | `--pitch-feedback` |

### Models

| Key | Type | Constraints / equivalent CLI |
| --- | --- | --- |
| `rknn_library` | Path string | `--rknn-library` |
| `model_path` | Path string | `--model` |
| `pose_model_path` | Path string | `--pose-model` |
| `pose_model_blue_path`, `pose_model_red_path` | Path string | `--pose-model-blue`, `--pose-model-red` |
| `input_size` | Integer | 1..8192; `--input-size` |
| `confidence`, `injured_confidence`, `nms` | Number | [0,1]; `--conf`, `--injured-conf`, `--nms` |
| `pose_confidence`, `pose_keypoint_confidence` | Number | [0,1]; `--pose-conf`, `--pose-kpt-conf` |
| `detect_core_mask`, `pose_core_mask` | Integer | -1, 0, 1, 2, 4; `--detect-core`, `--pose-core` |
| `parallel_inference`, `use_cuda` | Boolean | `--parallel-infer`, `--cuda` |
| `class_names` | String array | Nonempty; tensor order; no empty name, comma or surrounding whitespace; `--classes` |

Team-specific pose model selection keeps its existing precedence over the
generic model. `parallel_inference` still requires a pose model. A pose model
and `runtime.keypoints_file` are mutually exclusive.

### Calibration

| Key | Type | Constraints / equivalent CLI |
| --- | --- | --- |
| `calibration_file` | Path string | `--calibration` |
| `zone_geometry_file` | Path string | `--zone-geometry` |
| `task_calibration_file` | Path string | `--task-calibration` |
| `zone_color_file` | Path string | `--zone-color-calibration` |
| `pitch_presets_cdeg` | Three-integer array | FAR <= TRACK <= NEAR; each -4000..4000; `--pitch-presets` |
| `allow_mechanical_pitch_model` | Boolean | `--allow-mechanical-pitch-model` |
| `known_frame_angle` | Integer | 0 or 20; `--known-frame-angle` |
| `known_frame_id` | Integer | 1..255; `--known-frame-id` |

These paths reference the existing calibration formats. Measurement data is not
copied into runtime configuration or made valid by naming a runtime policy.

### Behavior

| Key | Type | Constraints / equivalent CLI |
| --- | --- | --- |
| `sensor_timeout_ms` | Integer | 1..60000; `--sensor-timeout-ms` |
| `tof_stop_distance_m` | Number | (0,100]; `--tof-stop-m` |
| `imu_tilt_limit_deg` | Number | (0,180]; `--imu-tilt-deg` |
| `match_seconds` | Integer | 1..86400; `--match-seconds` |
| `startup_advance_ms` | Integer | 0..60000; `--startup-advance-ms` |
| `startup_speed_mps` | Number | (0,1]; `--startup-speed` |
| `scan_wz_rps`, `turn_wz_rps` | Number | (0,3]; `--scan-wz`, `--turn-wz` |
| `clear_neighbours` | Boolean | `--clear-neighbours` |
| `search_cues` | Boolean | `--search-cues` |
| `controlled_ignore_clearance` | Boolean | `--controlled-ignore-clearance` |
| `controlled_empty_field` | Boolean | `--controlled-empty-field` |
| `assume_all_safe` | Boolean | `--assume-all-safe` |
| `assume_injured_trip` | Boolean | `--assume-injured-trip` |

Controlled and experimental requests retain their existing combination checks.
For example, `assume_all_safe` requires `demo_mode: "carry_once"` and
`controlled_empty_field: true`; `assume_injured_trip` also requires
`assume_all_safe`. They are explicit assumptions, not ordinary safety evidence.
The final resolved configuration records the enabled assumptions.

### Runtime

| Key | Type | Constraints / equivalent CLI |
| --- | --- | --- |
| `team` | String | `blue`, `red`; `--team` |
| `demo_mode` | String | `none`, `recognize`, `search`, `carry_once`; `--demo-mode` |
| `demo_seconds` | Integer | 1..60; `--demo-seconds` |
| `push_replay`, `geometry_replay`, `detect_image` | Path string | At most one nonempty operation; corresponding existing CLI flags |
| `keypoints_file` | Path string | `--keypoints-file` |
| `show` | Boolean | `false` maps to `--no-show` |
| `save_output`, `dry_run`, `hardware` | Boolean | `--save`, `--dry-run`, `--hardware` |
| `check_config` | Boolean | `--check-config` |
| `telemetry` | Boolean | `--telemetry` |
| `telemetry_fps` | Integer | 1..30; `--telemetry-fps` |
| `telemetry_file` | Path string | Absolute `/dev/shm/` snapshot path; `--telemetry-file` |
| `auto_run` | Boolean | `--auto-run` |
| `no_match_time_limit` | Boolean | `--no-match-time-limit` |
| `vision_log_dir` | Path string | `none` disables image logging; `--vision-log-dir` |
| `match_socket` | Path string | Resolved absolute Unix socket path shorter than 108 bytes; `--match-socket` |
| `require_instance_masks` | Boolean | `--require-masks` |

Live hardware output still requires IMU input and excludes replay/image modes,
dry-run and a separate read-only pitch receiver. Existing geometry and demo
requirements apply equally to file and CLI values. A combination may be
completed by explicit CLI arguments, because final combination checks run once
after both layers have been applied. File field types and bounds are checked
before CLI overrides; an invalid file value is never silently repaired by CLI.

## Scope and Development

The schema exposes existing active request parameters. It intentionally rejects
historical fields such as `detect_interval`, the old rectangle areas, old gripper
angles, tracker age and unused PnP thresholds. Adding a key requires identifying
its runtime consumer and final precedence first. There is no profile inheritance,
automatic file discovery or directory merge in schema v1.

The new application-layer loader uses the header-only `nlohmann_json` parser
for strict JSON types and duplicate-key validation. Install the development
package `nlohmann-json3-dev` (library version >= 3.6) as a build prerequisite where absent. The original
`src/config/config.cpp` remains free of that dependency and OpenCV.

`tests/config_loader_tests.cpp` verifies CLI compatibility, file-to-CLI
precedence, explicit booleans, derived model selection, source metadata, relative
paths, combination validation, strict schema errors and policy/operation
matching. These checks validate configuration behavior; camera, model and
physical execution evidence are separate acceptance criteria.
