#!/usr/bin/env python3
"""Verify explicit platform expectations without changing the frozen baseline."""

import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import record_baseline as baseline_tool


BASELINE = baseline_tool.ROOT / "tests/fixtures/runtime_legacy_baseline.json"
SUPPLEMENT = baseline_tool.ROOT / "tests/fixtures/runtime_legacy_aarch64.json"
PROTOCOL = baseline_tool.ROOT / "tests/fixtures/runtime_protocol_20261023.json"
BASELINE_SHA256 = "c828d652eb7316c817ab6182ba13f8202496b59f9b66b06dd9abdbd108b4d79f"


class PlatformBaselineTests(unittest.TestCase):
    def setUp(self):
        self.original_bytes = BASELINE.read_bytes()
        self.baseline = json.loads(self.original_bytes)
        self.supplement = json.loads(SUPPLEMENT.read_text())
        self.protocol = json.loads(PROTOCOL.read_text())
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.temporary = Path(self.directory.name)

    def tearDown(self):
        self.assertEqual(BASELINE.read_bytes(), self.original_bytes)
        self.assertEqual(hashlib.sha256(BASELINE.read_bytes()).hexdigest(), BASELINE_SHA256)

    def write_json(self, name, value):
        path = self.temporary / name
        path.write_text(json.dumps(value))
        return path

    def load(self, supplement=None, baseline_path=BASELINE, machine="aarch64"):
        path = SUPPLEMENT if supplement is None else self.write_json("supplement.json", supplement)
        return baseline_tool.load_baseline(baseline_path, path, machine=machine)

    def load_protocol(self, protocol=None, baseline_path=BASELINE):
        path = PROTOCOL if protocol is None else self.write_json("protocol.json", protocol)
        return baseline_tool.load_baseline(baseline_path, protocol_baseline=path)

    def failures(self, actual, expected):
        expected = copy.deepcopy(expected)
        expected["resources_sha256"] = {}
        return baseline_tool.comparison_failures(actual, expected)

    def test_default_comparison_is_unchanged_for_every_architecture(self):
        for machine in ("x86_64", "aarch64", "unknown"):
            with self.subTest(machine=machine):
                expected = baseline_tool.load_baseline(BASELINE, machine=machine)
                self.assertEqual(expected, self.baseline)
                self.assertEqual(self.failures(self.baseline["cases"], expected), [])
                actual = self.load()["cases"]
                self.assertEqual(self.failures(actual, expected), ["geometry: stdout differs"])

    def test_protocol_supplement_changes_only_the_declared_count(self):
        expected = self.load_protocol()
        actual = copy.deepcopy(self.baseline)
        actual["cases"]["push_delivery"]["stdout"] = actual["cases"]["push_delivery"]["stdout"].replace(
            self.protocol["replacements"][0]["from"], self.protocol["replacements"][0]["to"])
        self.assertEqual(expected, actual)
        self.assertEqual(expected["cases"]["geometry"], self.baseline["cases"]["geometry"])
        self.assertEqual(self.failures(expected["cases"], expected), [])

    def test_protocol_and_platform_supplements_compose_in_order(self):
        expected = baseline_tool.load_baseline(
            BASELINE, SUPPLEMENT, machine="aarch64", protocol_baseline=PROTOCOL)
        self.assertEqual(expected["cases"]["push_delivery"]["stdout"].count("camera_protocol_cmd_deg=40"), 0)
        self.assertEqual(expected["cases"]["push_delivery"]["stdout"].count("camera_protocol_cmd_deg=30"), 55)
        self.assertIn(self.supplement["geometry_stdout"]["to"], expected["cases"]["geometry"]["stdout"])
        self.assertEqual(self.failures(expected["cases"], expected), [])

    def test_protocol_provenance_and_counts_are_strict(self):
        mutations = (
            ("schema_version", 2), ("schema_version", True), ("protocol", ""),
            ("baseline_sha256", "0" * 64), ("baseline_sha256", "invalid"),
            ("legacy_revision", "0" * 40), ("evidence", []), ("evidence", [""]),
            ("replacements", []),
        )
        for key, value in mutations:
            with self.subTest(key=key, value=value):
                protocol = copy.deepcopy(self.protocol)
                protocol[key] = value
                with self.assertRaises(ValueError):
                    self.load_protocol(protocol)
        for key in self.protocol:
            with self.subTest(missing=key):
                protocol = copy.deepcopy(self.protocol)
                del protocol[key]
                with self.assertRaises(ValueError):
                    self.load_protocol(protocol)
        for key, value in (("case", "missing"), ("field", "stderr"),
                           ("from", ""), ("to", ""), ("count", 0), ("count", 54),
                           ("count", 56), ("count", True)):
            with self.subTest(replacement=key, value=value):
                protocol = copy.deepcopy(self.protocol)
                protocol["replacements"][0][key] = value
                with self.assertRaises(ValueError):
                    self.load_protocol(protocol)
        for key in ("case", "field", "from", "to", "count"):
            with self.subTest(missing_replacement_key=key):
                protocol = copy.deepcopy(self.protocol)
                del protocol["replacements"][0][key]
                with self.assertRaises(ValueError):
                    self.load_protocol(protocol)

    def test_protocol_supplement_cannot_expand_scope(self):
        for key in ("cases", "push_delivery", "args", "returncode", "stderr", "resources_sha256"):
            with self.subTest(key=key):
                protocol = copy.deepcopy(self.protocol)
                protocol[key] = {}
                with self.assertRaises(ValueError):
                    self.load_protocol(protocol)
        protocol = copy.deepcopy(self.protocol)
        protocol["replacements"][0]["case"] = "geometry"
        with self.assertRaisesRegex(ValueError, "exactly 55 times"):
            self.load_protocol(protocol)
        protocol = copy.deepcopy(self.protocol)
        protocol["replacements"][0]["from"] = "frame_offset_deg=20"
        protocol["replacements"][0]["count"] = 1
        with self.assertRaisesRegex(ValueError, "exactly 1 times"):
            self.load_protocol(protocol)

    def test_protocol_supplement_is_not_applied_to_recording(self):
        output = self.temporary / "record.json"
        result = subprocess.run([sys.executable, str(Path(baseline_tool.__file__)), "missing-binary",
                                 "--record", str(output), "--protocol-baseline", str(PROTOCOL)],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 2)
        self.assertIn("--protocol-baseline is only allowed with --compare", result.stderr)
        self.assertFalse(output.exists())

    def test_supplement_changes_only_the_verified_geometry_line(self):
        expected = self.load()
        self.assertEqual(self.failures(expected["cases"], expected), [])
        expected["cases"]["geometry"]["stdout"] = expected["cases"]["geometry"]["stdout"].replace(
            self.supplement["geometry_stdout"]["to"], self.supplement["geometry_stdout"]["from"], 1)
        self.assertEqual(expected, self.baseline)

    def test_supplement_records_the_verified_original_revision_and_binary(self):
        self.assertEqual(self.supplement["baseline_sha256"], BASELINE_SHA256)
        self.assertEqual(self.supplement["legacy_revision"], self.baseline["revision"])
        self.assertEqual(self.supplement["legacy_binary_sha256"],
                         "2be9928092d725eb089f98a384f522e57b65dc24cc2e789cf05f2b0a800ad582")
        self.assertTrue((baseline_tool.ROOT / self.supplement["evidence"][0]).is_file())

    def test_wrong_machine_is_rejected(self):
        for machine in ("x86_64", "arm64", "", "unknown"):
            with self.subTest(machine=machine), self.assertRaisesRegex(ValueError, "machine"):
                self.load(machine=machine)

    def test_schema_and_provenance_are_strict(self):
        mutations = (
            ("schema_version", 2), ("schema_version", True),
            ("platform_machine", "x86_64"), ("baseline_sha256", "0" * 64),
            ("baseline_sha256", "invalid"), ("legacy_revision", "0" * 40),
            ("legacy_binary_sha256", "invalid"), ("evidence", []),
            ("evidence", [""]), ("evidence", "report.md"),
        )
        for key, value in mutations:
            with self.subTest(key=key, value=value):
                supplement = copy.deepcopy(self.supplement)
                supplement[key] = value
                with self.assertRaises(ValueError):
                    self.load(supplement)
        for key in self.supplement:
            with self.subTest(missing=key):
                supplement = copy.deepcopy(self.supplement)
                del supplement[key]
                with self.assertRaises(ValueError):
                    self.load(supplement)

    def test_other_cases_and_fields_cannot_be_overridden(self):
        for key in ("cases", "push_delivery", "args", "returncode", "stderr"):
            with self.subTest(key=key):
                supplement = copy.deepcopy(self.supplement)
                supplement[key] = {}
                with self.assertRaises(ValueError):
                    self.load(supplement)
                supplement = copy.deepcopy(self.supplement)
                supplement["geometry_stdout"][key] = {}
                with self.assertRaises(ValueError):
                    self.load(supplement)

    def test_duplicate_json_keys_are_rejected(self):
        for key, value in (("schema_version", "1"), ("from", '"duplicate"')):
            with self.subTest(key=key):
                content = json.dumps(self.supplement)
                content = content.replace(f'"{key}":', f'"{key}": {value}, "{key}":', 1)
                path = self.temporary / "duplicate-keys.json"
                path.write_text(content)
                with self.assertRaisesRegex(ValueError, "duplicate key"):
                    baseline_tool.load_baseline(BASELINE, path, machine="aarch64")

    def test_only_the_complete_first_frame_residual_line_can_be_declared(self):
        line = self.supplement["geometry_stdout"]["to"]
        invalid = (line.rstrip("\n"), line + line, line.replace("frame=1", "frame=2"),
                   line.replace("zone_valid=1", "zone_valid=0"), line.replace("points=6", "points=5"),
                   line.replace("reason=ok", "reason=failed"), line.replace("subzone=supply", "subzone=unavailable"),
                   line.replace("7.72043e-08", "nan"), line.replace("7.72043e-08", "inf"))
        for value in invalid:
            with self.subTest(line=value):
                supplement = copy.deepcopy(self.supplement)
                supplement["geometry_stdout"]["to"] = value
                with self.assertRaises(ValueError):
                    self.load(supplement)
        supplement = copy.deepcopy(self.supplement)
        supplement["geometry_stdout"]["to"] = supplement["geometry_stdout"]["from"]
        with self.assertRaises(ValueError):
            self.load(supplement)

    def test_missing_and_duplicate_source_lines_are_rejected(self):
        supplement = copy.deepcopy(self.supplement)
        supplement["geometry_stdout"]["from"] = supplement["geometry_stdout"]["from"].replace(
            "7.81572e-08", "7.81573e-08")
        with self.assertRaisesRegex(ValueError, "exactly once"):
            self.load(supplement)
        duplicate = copy.deepcopy(self.baseline)
        duplicate["cases"]["geometry"]["stdout"] += self.supplement["geometry_stdout"]["from"]
        path = self.write_json("duplicated-baseline.json", duplicate)
        supplement = copy.deepcopy(self.supplement)
        supplement["baseline_sha256"] = baseline_tool.digest(path)
        with self.assertRaisesRegex(ValueError, "exactly once"):
            self.load(supplement, baseline_path=path)

    def test_every_actual_case_field_still_requires_exact_equality(self):
        expected = self.load()
        for case_name in expected["cases"]:
            for field in ("args", "returncode", "stdout", "stderr"):
                with self.subTest(case=case_name, field=field):
                    actual = copy.deepcopy(expected["cases"])
                    value = actual[case_name][field]
                    actual[case_name][field] = value + (["--changed"] if field == "args" else
                                                      1 if field == "returncode" else "changed")
                    self.assertEqual(self.failures(actual, expected), [f"{case_name}: {field} differs"])

    def test_other_residuals_and_even_tiny_changes_still_fail(self):
        expected = self.load()
        stdout = expected["cases"]["geometry"]["stdout"]
        for before, after in (("7.72043e-08", "7.72044e-08"), ("7.72043e-08", "7.81572e-08"),
                              ("2.35608e-08", "2.35609e-08"), ("residual_m=inf", "residual_m=0")):
            with self.subTest(before=before, after=after):
                actual = copy.deepcopy(expected["cases"])
                actual["geometry"]["stdout"] = stdout.replace(before, after, 1)
                self.assertEqual(self.failures(actual, expected), ["geometry: stdout differs"])
        for replacement in ("", self.supplement["geometry_stdout"]["to"] * 2):
            with self.subTest(replacement=replacement):
                actual = copy.deepcopy(expected["cases"])
                actual["geometry"]["stdout"] = stdout.replace(self.supplement["geometry_stdout"]["to"], replacement, 1)
                self.assertEqual(self.failures(actual, expected), ["geometry: stdout differs"])

    def test_resource_hashes_are_preserved_and_still_checked(self):
        expected = self.load()
        self.assertEqual(expected["resources_sha256"], self.baseline["resources_sha256"])
        (self.temporary / "resource").write_text("original")
        expected["resources_sha256"] = {"resource": baseline_tool.digest(self.temporary / "resource")}
        self.assertEqual(baseline_tool.comparison_failures(expected["cases"], expected, self.temporary), [])
        (self.temporary / "resource").write_text("changed")
        self.assertEqual(baseline_tool.comparison_failures(expected["cases"], expected, self.temporary),
                         ["resource: baseline resource differs"])

    def test_retired_config_resources_are_checked_at_their_fixture_paths(self):
        for original, archived in baseline_tool.LEGACY_RESOURCE_PATHS.items():
            with self.subTest(resource=original):
                resource = baseline_tool.ROOT / archived
                if original == "config/camera.yaml":
                    # The active path now contains the latest calibration, not frozen bytes.
                    self.assertNotEqual(baseline_tool.digest(baseline_tool.ROOT / original),
                                        baseline_tool.digest(resource))
                elif original.startswith("config/"):
                    self.assertFalse((baseline_tool.ROOT / original).exists())
                # Archived bytes must match the frozen resource, including legacy protocol fixtures.
                self.assertEqual(baseline_tool.digest(resource), self.baseline["resources_sha256"][original])
                self.assertIn(original, self.baseline["resources_sha256"])
                expected = {"cases": {}, "resources_sha256": {
                    original: baseline_tool.digest(resource)}}
                fixture = self.temporary / archived
                fixture.parent.mkdir(parents=True, exist_ok=True)
                fixture.write_bytes(resource.read_bytes())
                self.assertEqual(baseline_tool.comparison_failures({}, expected, self.temporary), [])
                fixture.write_text("changed")
                failure = [f"{original}: baseline resource differs"]
                self.assertEqual(baseline_tool.comparison_failures({}, expected, self.temporary), failure)
                fixture.unlink()
                self.assertEqual(baseline_tool.comparison_failures({}, expected, self.temporary), failure)

    def test_record_cannot_use_platform_expectations(self):
        output = self.temporary / "record.json"
        result = subprocess.run([sys.executable, str(Path(baseline_tool.__file__)), "missing-binary",
                                 "--record", str(output), "--platform-baseline", str(SUPPLEMENT)],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 2)
        self.assertIn("--platform-baseline is only allowed with --compare", result.stderr)
        self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
