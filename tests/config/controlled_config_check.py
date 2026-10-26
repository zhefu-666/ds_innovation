#!/usr/bin/env python3
"""Check the intended static rejection without accepting missing-resource errors."""
from pathlib import Path
import subprocess
import sys

binary, root = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
project = str(root / "config/project.json")
result = subprocess.run([str(binary), "--check-config", "--controlled-empty-field", "--team", "blue",
                         "--calibration", str(root / "config/camera.yaml"),
                         "--zone-geometry", project + "#/zone_geometry",
                         "--task-calibration", project + "#/task_calibration/runtime",
                         "--pose-model-blue", "models/zone_pose_fp.rknn"],
                        cwd=root, capture_output=True, text=True, timeout=15)
assert result.returncode == 2, (result.returncode, result.stdout, result.stderr)
assert "OK task_calibration" in result.stdout, result.stdout
assert "MISSING new_frame_mapping_and_required_calibration" in result.stdout, result.stdout
assert "STATIC_ONLY:" in result.stdout and "hardware_output=disabled" in result.stdout, result.stdout
assert not result.stderr, result.stderr
print("Controlled configuration rejects missing readiness with all resources loaded")
