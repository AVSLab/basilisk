# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""Unit tests for the Basilisk usage-metrics collector."""

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "collect_usage_metrics.py"
SPEC = importlib.util.spec_from_file_location("collect_usage_metrics", SCRIPT_PATH)
METRICS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(METRICS)


def _github_metrics():
    return {
        "clones": [
            {
                "date": "2026-09-02",
                "github_clones": 12,
                "github_unique_cloners": 5,
            },
            {
                "date": "2026-09-03",
                "github_clones": 20,
                "github_unique_cloners": 7,
            },
        ],
        "clone_window_count": 32,
        "clone_window_unique_cloners": 10,
        "forks": 133,
        "release_asset_downloads": 4,
    }


class UsageMetricsTest(unittest.TestCase):
    def test_merge_history_preserves_and_updates_overlapping_data(self):
        """An API overlap updates known dates without discarding older counts."""
        existing_rows = [
            {
                **METRICS._empty_history_row("2026-09-01"),
                "github_clones": 8,
                "github_unique_cloners": 4,
            },
            {
                **METRICS._empty_history_row("2026-09-02"),
                "github_clones": 1,
                "github_unique_cloners": 1,
            },
        ]
        pypi_rows = [
            {
                "date": "2026-09-02",
                "pypi_downloads": 40,
                "pypi_pip_downloads": 25,
            },
            {
                "date": "2026-09-03",
                "pypi_downloads": 50,
                "pypi_pip_downloads": 30,
            },
        ]

        rows = METRICS.merge_history(
            existing_rows,
            pypi_rows,
            _github_metrics(),
            "2026-09-03",
        )

        self.assertEqual(
            [row["date"] for row in rows],
            [
                "2026-09-01",
                "2026-09-02",
                "2026-09-03",
            ],
        )
        self.assertEqual(rows[0]["github_clones"], 8)
        self.assertEqual(rows[1]["github_clones"], 12)
        self.assertEqual(rows[2]["github_forks"], 133)
        self.assertEqual(rows[2]["github_release_asset_downloads"], 4)

    def test_summary_does_not_sum_daily_unique_cloners(self):
        """The summary uses GitHub's window-level unique-cloner value."""
        rows = METRICS.merge_history(
            [],
            [
                {
                    "date": "2026-09-03",
                    "pypi_downloads": 50,
                    "pypi_pip_downloads": 30,
                }
            ],
            _github_metrics(),
            "2026-09-03",
        )

        summary = METRICS.build_summary(
            rows,
            "bsk",
            "AVSLab/basilisk",
            _github_metrics(),
            "2026-09-04T00:00:00+00:00",
        )

        self.assertEqual(summary["github"]["tracked_clones"], 32)
        self.assertEqual(
            summary["github"]["current_clone_window"]["unique_cloners"],
            10,
        )
        self.assertNotIn("unique_cloners", summary["github"])

    def test_csv_history_round_trip_preserves_missing_values(self):
        """Blank cells remain missing when CSV history is reloaded."""
        rows = [
            {
                **METRICS._empty_history_row("2026-09-03"),
                "pypi_downloads": 50,
                "pypi_pip_downloads": 30,
            }
        ]
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "metrics.csv"
            METRICS.write_history(path, rows)
            loaded_rows = METRICS.load_history(path)

        self.assertEqual(loaded_rows, rows)

    def test_generated_readme_explains_metric_limitations(self):
        """The generated landing page distinguishes downloads from installs."""
        rows = METRICS.merge_history(
            [],
            [
                {
                    "date": "2026-09-03",
                    "pypi_downloads": 50,
                    "pypi_pip_downloads": 30,
                }
            ],
            _github_metrics(),
            "2026-09-03",
        )
        summary = METRICS.build_summary(
            rows,
            "bsk",
            "AVSLab/basilisk",
            _github_metrics(),
            "2026-09-04T00:00:00+00:00",
        )

        readme = METRICS.render_readme(summary)

        self.assertIn("file-download events, not verified installations", readme)
        self.assertIn("latest 14 days", readme)
        self.assertIn("automatically generated source ZIP", readme)

    def test_summary_is_json_serializable(self):
        """The generated summary contains only stable JSON-compatible values."""
        rows = METRICS.merge_history(
            [],
            [],
            _github_metrics(),
            "2026-09-03",
        )
        summary = METRICS.build_summary(
            rows,
            "bsk",
            "AVSLab/basilisk",
            _github_metrics(),
            "2026-09-04T00:00:00+00:00",
        )

        self.assertEqual(json.loads(json.dumps(summary)), summary)

    def test_usage_svg_contains_accessible_labeled_series(self):
        """The generated chart identifies its content and plotted series."""
        rows = METRICS.merge_history(
            [],
            [
                {
                    "date": "2026-09-03",
                    "pypi_downloads": 50,
                    "pypi_pip_downloads": 30,
                }
            ],
            _github_metrics(),
            "2026-09-03",
        )

        svg = METRICS.render_usage_svg(rows, "2026-09-04T00:00:00+00:00")

        self.assertIn('role="img"', svg)
        self.assertIn('<title id="chart-title">', svg)
        self.assertIn("7-day trailing mean", svg)
        self.assertIn('class="data-line pypi-downloads"', svg)
        self.assertIn('class="data-line github-clones"', svg)


if __name__ == "__main__":
    unittest.main()
