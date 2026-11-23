#!/usr/bin/env python3

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "coverage_ratchet.py"


class CoverageRatchetTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.baseline = self.root / "baseline.json"
        self.summary = self.root / "summary.json"
        self.baseline.write_text(
            json.dumps(
                {
                    "schema": 1,
                    "line": {"covered": 9, "total": 10},
                    "branch": {"covered": 4, "total": 5},
                }
            ),
            encoding="utf-8",
        )

    def tearDown(self):
        self.temporary_directory.cleanup()

    def write_summary(self, line_covered=90, branch_covered=80):
        self.summary.write_text(
            json.dumps(
                {
                    "line_covered": line_covered,
                    "line_total": 100,
                    "branch_covered": branch_covered,
                    "branch_total": 100,
                }
            ),
            encoding="utf-8",
        )

    def run_ratchet(self, action):
        return subprocess.run(
            [sys.executable, str(SCRIPT), action, str(self.baseline), str(self.summary)],
            check=False,
            capture_output=True,
            text=True,
        )

    def test_check_accepts_equal_or_higher_ratios(self):
        self.write_summary(line_covered=91, branch_covered=80)
        result = self.run_ratchet("check")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Coverage ratchet passed", result.stdout)

    def test_check_rejects_any_regression(self):
        self.write_summary(line_covered=89, branch_covered=79)
        result = self.run_ratchet("check")
        self.assertEqual(result.returncode, 1)
        self.assertIn("line coverage regressed", result.stdout)
        self.assertIn("branch coverage regressed", result.stdout)

    def test_update_records_exact_counts_only_after_improvement(self):
        self.write_summary(line_covered=92, branch_covered=81)
        result = self.run_ratchet("update")
        self.assertEqual(result.returncode, 0, result.stderr)
        updated = json.loads(self.baseline.read_text(encoding="utf-8"))
        self.assertEqual(updated["line"], {"covered": 92, "total": 100})
        self.assertEqual(updated["branch"], {"covered": 81, "total": 100})

    def test_update_refuses_to_lower_either_metric(self):
        original = self.baseline.read_text(encoding="utf-8")
        self.write_summary(line_covered=91, branch_covered=79)
        result = self.run_ratchet("update")
        self.assertEqual(result.returncode, 1)
        self.assertEqual(self.baseline.read_text(encoding="utf-8"), original)

    def test_invalid_summary_fails_closed(self):
        self.summary.write_text('{"line_covered": 1}', encoding="utf-8")
        result = self.run_ratchet("check")
        self.assertEqual(result.returncode, 2)
        self.assertIn("expected a positive integer", result.stderr)


if __name__ == "__main__":
    unittest.main()
