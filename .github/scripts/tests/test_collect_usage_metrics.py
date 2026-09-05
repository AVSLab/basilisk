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
import os
import shutil
import sqlite3
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from datetime import date, datetime, timezone
from http.client import HTTPResponse, IncompleteRead, RemoteDisconnected
from io import BytesIO
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from urllib.error import HTTPError, URLError


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


def _http_response(body, *, declared_length=None):
    """Parse an in-memory HTTP response, optionally with a truncated body."""
    length = len(body) if declared_length is None else declared_length
    wire_data = f"HTTP/1.1 200 OK\r\nContent-Length: {length}\r\n\r\n".encode("ascii") + body
    socket = SimpleNamespace(makefile=lambda mode: BytesIO(wire_data))
    response = HTTPResponse(socket)
    response.begin()
    return response


class HttpRequestTest(unittest.TestCase):
    def test_truncated_response_retries_and_can_recover(self):
        """An incomplete body is discarded and a later complete response is used."""
        responses = [_http_response(b"{", declared_length=2), _http_response(b"{}")]
        with patch.object(METRICS, "urlopen", side_effect=responses) as request, \
                patch.object(METRICS.time, "sleep") as sleep:
            self.assertEqual(METRICS._request_text("https://example.invalid/metrics"), "{}")
        self.assertEqual(request.call_count, 2)
        sleep.assert_called_once_with(METRICS.RETRY_DELAY_SECONDS)

    def test_truncated_responses_exhaust_retries_as_metrics_error(self):
        """Real HTTP body truncation becomes a source error after all retries."""
        responses = [_http_response(b"{", declared_length=2) for _ in range(METRICS.REQUEST_ATTEMPTS)]
        with patch.object(METRICS, "urlopen", side_effect=responses) as request, \
                patch.object(METRICS.time, "sleep") as sleep:
            with self.assertRaises(METRICS.MetricsError) as raised:
                METRICS._request_text("https://example.invalid/metrics")
        self.assertIsInstance(raised.exception.__cause__, IncompleteRead)
        self.assertEqual(request.call_count, METRICS.REQUEST_ATTEMPTS)
        self.assertEqual(
            [call.args[0] for call in sleep.call_args_list],
            [METRICS.RETRY_DELAY_SECONDS*(attempt+1) for attempt in range(METRICS.REQUEST_ATTEMPTS-1)],
        )

    def test_response_read_errors_are_retried_and_normalized(self):
        """Socket and HTTP read failures use the same bounded retry path."""
        for error in (ConnectionResetError("reset"), TimeoutError("timed out"),
                      OSError("read failed"), URLError("unreachable"), RemoteDisconnected("disconnected")):
            with self.subTest(error=type(error).__name__):
                responses = [_http_response(b"{}") for _ in range(METRICS.REQUEST_ATTEMPTS)]
                with patch.object(METRICS, "urlopen", side_effect=responses) as request, \
                        patch.object(HTTPResponse, "read", side_effect=error), \
                        patch.object(METRICS.time, "sleep") as sleep:
                    with self.assertRaises(METRICS.MetricsError) as raised:
                        METRICS._request_text("https://example.invalid/metrics")
                self.assertIs(raised.exception.__cause__, error)
                self.assertEqual(request.call_count, METRICS.REQUEST_ATTEMPTS)
                self.assertEqual(sleep.call_count, METRICS.REQUEST_ATTEMPTS-1)

    def test_non_retryable_http_status_still_fails_immediately(self):
        """HTTP status handling takes precedence over the generic I/O handler."""
        error = HTTPError("https://example.invalid/metrics", 403, "Forbidden", {}, BytesIO())
        self.addCleanup(error.close)
        with patch.object(METRICS, "urlopen", side_effect=error) as request, \
                patch.object(METRICS.time, "sleep") as sleep:
            with self.assertRaisesRegex(METRICS.MetricsError, "HTTP 403"):
                METRICS._request_text("https://example.invalid/metrics")
        request.assert_called_once()
        sleep.assert_not_called()


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
        rows = [
            {"date": f"2026-09-{day:02d}", "pypi_downloads": 50,
             "pypi_pip_downloads": 30, "github_clones": 12, "github_unique_cloners": 5}
            for day in range(1, 9)
        ]

        svg = METRICS.render_usage_svg(rows, "2026-09-04T00:00:00+00:00")

        self.assertIn('role="img"', svg)
        self.assertIn('<title id="chart-title">', svg)
        self.assertIn("7-day trailing mean", svg)
        self.assertIn('class="data-line pypi-downloads"', svg)
        self.assertIn('class="data-line github-clones"', svg)
        ET.fromstring(svg)

    def test_rolling_average_counts_explicit_zero_days(self):
        """Seven known days include zero counts in the denominator."""
        rows = [
            {"date": f"2026-09-{day:02d}", "pypi_downloads": 70 if day in (1, 7) else 0}
            for day in range(1, 8)
        ]
        self.assertEqual(METRICS._rolling_series(rows, "pypi_downloads"), [
            (date(2026, 9, 7).toordinal(), 20.0)
        ])

    def test_rolling_average_requires_a_complete_window(self):
        """Both absent dates and explicitly unknown values break the window."""
        complete = [
            {"date": f"2026-09-{day:02d}", "pypi_downloads": day}
            for day in range(1, 16)
        ]
        for missing in (True, False):
            with self.subTest(absent=missing):
                rows = [dict(row) for row in complete]
                if missing:
                    del rows[7]
                else:
                    rows[7]["pypi_downloads"] = None
                self.assertEqual(METRICS._rolling_series(rows, "pypi_downloads"), [
                    (date(2026, 9, 7).toordinal(), 4.0),
                    (date(2026, 9, 15).toordinal(), 12.0),
                ])

    def test_sparse_history_remains_unknown_in_csv_and_chart(self):
        """An absent source date is retained as blank and never averaged as zero."""
        rows = METRICS.merge_history([], [
            {"date": "2026-09-01", "pypi_downloads": 70, "pypi_pip_downloads": 40},
            {"date": "2026-09-07", "pypi_downloads": 70, "pypi_pip_downloads": 40},
        ], None, "2026-09-07")
        self.assertEqual(len(rows), 7)
        self.assertIsNone(rows[1]["pypi_downloads"])
        self.assertEqual(METRICS._rolling_series(rows, "pypi_downloads"), [])
        self.assertIn("No complete seven-day window", METRICS.render_usage_svg(rows, "test"))

    def test_pypi_query_filters_known_files_and_retains_unknown_filenames(self):
        """Evaluate the actual SQL predicate against distribution and metadata fixtures."""
        class CountIf:
            def __init__(self):
                self.total = 0

            def step(self, value):
                self.total += bool(value)

            def finalize(self):
                return self.total

        with sqlite3.connect(":memory:") as connection:
            self.addCleanup(connection.close)
            connection.row_factory = sqlite3.Row
            connection.create_function("endsWith", 2, lambda value, suffix: value.endswith(suffix))
            connection.create_aggregate("countIf", 1, CountIf)
            connection.execute("CREATE TABLE pypi (date TEXT, project TEXT, installer TEXT, filename TEXT)")
            connection.executemany("INSERT INTO pypi VALUES (?, ?, ?, ?)", [
                ("2026-09-01", "bsk", "pip", filename)
                for filename in ("bsk.whl", "bsk.tar.gz", "bsk.zip", "", "bsk.whl.metadata", "bsk.whl.asc")
            ] + [
                ("2026-09-01", "bsk", "uv", "bsk.whl"),
                ("2026-09-01", "bsk", "bandersnatch", "bsk.whl"),
                ("2026-09-01", "another-package", "pip", "other.whl"),
                ("2026-09-02", "bsk", "pip", "bsk.whl.metadata"),
            ])

            def query_source(url, *, data, headers):
                # Translate only ClickHouse's expression-alias WITH syntax and
                # wire formatting; execute its filtering/counting expressions.
                prefix, select = data.decode().split("SELECT", 1)
                expression = prefix.strip()[len("WITH "):].rsplit(" AS ", 1)[0]
                sql = f"WITH annotated AS (SELECT *, {expression} AS distribution_or_unknown FROM pypi) SELECT {select}"
                sql = sql.replace("FROM pypi.pypi", "FROM annotated")
                sql = sql.replace("{package:String}", "?").replace("FORMAT JSONEachRow", "")
                return "\n".join(json.dumps(dict(row)) for row in connection.execute(sql, ("bsk",)))

            with patch.object(METRICS, "_request_text", side_effect=query_source):
                rows = METRICS.fetch_pypi_history("bsk")

        self.assertEqual(rows, [
            {"date": "2026-09-01", "pypi_downloads": 5, "pypi_pip_downloads": 4},
            {"date": "2026-09-02", "pypi_downloads": 0, "pypi_pip_downloads": 0},
        ])


class SourceFailureTest(unittest.TestCase):
    def setUp(self):
        """Seed retained artifacts from a successful collection."""
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.output = Path(temporary.name)
        self.today = "2026-09-04"
        self.yesterday = "2026-09-03"
        clock = patch.object(METRICS, "datetime")
        self.addCleanup(clock.stop)
        clock.start().now.return_value = datetime(2026, 9, 4, tzinfo=timezone.utc)
        self.old_time = self.yesterday + "T00:00:00+00:00"
        self.pypi_rows = [{"date": self.today, "pypi_downloads": 40, "pypi_pip_downloads": 20}]
        self.github = _github_metrics()
        self.github["clones"] = [{"date": self.today, "github_clones": 32, "github_unique_cloners": 10}]
        self.old_rows = [{
            **METRICS._empty_history_row(self.yesterday),
            "pypi_downloads": 100, "pypi_pip_downloads": 50,
            "github_clones": 99, "github_unique_cloners": 20,
            "github_forks": 120, "github_release_asset_downloads": 2,
        }]
        METRICS.write_history(self.output / "metrics.csv", self.old_rows)
        sources = {name: {"status": "ok", "last_attempt_at": self.old_time,
                          "last_success_at": self.old_time, "error": None}
                   for name in ("pypi", "github")}
        self.previous = METRICS.build_summary(
            self.old_rows, "bsk", "AVSLab/basilisk", self.github, self.old_time, sources,
        )
        (self.output / "summary.json").write_text(json.dumps(self.previous), encoding="utf-8")

    def collect(self, *, pypi_error=None, github_error=None, output=None):
        with patch.object(METRICS, "fetch_pypi_history", return_value=self.pypi_rows,
                          side_effect=pypi_error) as pypi, \
                patch.object(METRICS, "fetch_github_metrics", return_value=self.github,
                             side_effect=github_error) as github:
            summary = METRICS.collect_metrics(output or self.output, "bsk", "AVSLab/basilisk", "fixture")
            pypi.assert_called_once()
            github.assert_called_once()
            return summary

    def test_pypi_failure_still_retains_github_and_marks_pypi_stale(self):
        """A PyPI outage publishes new clones without relabeling old PyPI observations."""
        summary = self.collect(pypi_error=METRICS.MetricsError("ClickPy unavailable"))
        self.assertEqual(summary["collection_status"], "partial")
        self.assertEqual(summary["github"]["tracked_clones"], 131)
        self.assertEqual(summary["pypi"]["downloads_excluding_known_mirrors"], 100)
        self.assertEqual(summary["sources"]["pypi"]["last_success_at"], self.old_time)
        self.assertEqual(summary["sources"]["github"]["last_success_at"], summary["generated_at"])
        self.assertIsNone(METRICS.load_history(self.output / "metrics.csv")[-1]["pypi_downloads"])
        self.assertIn("ClickPy unavailable", (self.output / "README.md").read_text())
        self.assertIn("pypi: error", (self.output / "usage.svg").read_text())

    def test_github_failure_preserves_window_without_fabricating_a_snapshot(self):
        """Old GitHub totals and unique-window data remain explicitly stale."""
        summary = self.collect(github_error=METRICS.MetricsError("GitHub unavailable"))
        self.assertEqual(summary["pypi"]["downloads_excluding_known_mirrors"], 140)
        self.assertEqual(summary["github"]["forks"], 120)
        self.assertEqual(summary["github"]["current_clone_window"], self.previous["github"]["current_clone_window"])
        self.assertEqual(summary["sources"]["github"]["last_success_at"], self.old_time)
        latest = METRICS.load_history(self.output / "metrics.csv")[-1]
        self.assertIsNone(latest["github_forks"])
        self.assertIsNone(latest["github_clones"])

    def test_truncated_pypi_response_still_retains_github(self):
        """A body-read failure saves new GitHub data and retains stale PyPI history."""
        responses = [_http_response(b"{", declared_length=2) for _ in range(METRICS.REQUEST_ATTEMPTS)]
        with patch.object(METRICS, "fetch_github_metrics", return_value=self.github) as github, \
                patch.object(METRICS, "urlopen", side_effect=responses) as request, \
                patch.object(METRICS.time, "sleep"):
            summary = METRICS.collect_metrics(self.output, "bsk", "AVSLab/basilisk", "fixture")
        github.assert_called_once()
        self.assertEqual(request.call_count, METRICS.REQUEST_ATTEMPTS)
        self.assertEqual(summary["collection_status"], "partial")
        self.assertEqual(summary["sources"]["pypi"]["status"], "error")
        self.assertEqual(summary["sources"]["pypi"]["last_success_at"], self.old_time)
        self.assertEqual(summary["sources"]["github"]["status"], "ok")
        rows = METRICS.load_history(self.output / "metrics.csv")
        self.assertEqual(rows[0], self.old_rows[0])
        self.assertEqual(rows[-1]["github_clones"], 32)
        self.assertIsNone(rows[-1]["pypi_downloads"])
        self.assertEqual(json.loads((self.output / "summary.json").read_text()), summary)
        self.assertIn("IncompleteRead", (self.output / "README.md").read_text())
        self.assertIn("pypi: error", (self.output / "usage.svg").read_text())

    def test_truncated_github_response_still_collects_pypi(self):
        """An earlier GitHub body-read failure does not skip PyPI collection."""
        responses = [_http_response(b"{", declared_length=2) for _ in range(METRICS.REQUEST_ATTEMPTS)]
        with patch.object(METRICS, "fetch_pypi_history", return_value=self.pypi_rows) as pypi, \
                patch.object(METRICS, "urlopen", side_effect=responses) as request, \
                patch.object(METRICS.time, "sleep"):
            summary = METRICS.collect_metrics(self.output, "bsk", "AVSLab/basilisk", "fixture")
        pypi.assert_called_once()
        self.assertEqual(request.call_count, METRICS.REQUEST_ATTEMPTS)
        self.assertEqual(summary["collection_status"], "partial")
        self.assertEqual(summary["sources"]["github"]["status"], "error")
        self.assertEqual(summary["sources"]["github"]["last_success_at"], self.old_time)
        self.assertEqual(summary["sources"]["pypi"]["status"], "ok")
        rows = METRICS.load_history(self.output / "metrics.csv")
        self.assertEqual(rows[0], self.old_rows[0])
        self.assertEqual(rows[-1]["pypi_downloads"], 40)
        self.assertIsNone(rows[-1]["github_clones"])

    def test_first_partial_collection_reports_unknown_counts_not_zero(self):
        """An unavailable source without retained history is represented by nulls."""
        for failed_source in ("pypi", "github"):
            with self.subTest(source=failed_source):
                output = self.output / failed_source
                summary = self.collect(output=output, **{
                    failed_source + "_error": METRICS.MetricsError("unavailable")
                })
                self.assertIsNone(summary["sources"][failed_source]["last_success_at"])
                if failed_source == "pypi":
                    self.assertIsNone(summary["pypi"]["downloads_excluding_known_mirrors"])
                else:
                    self.assertIsNone(summary["github"]["forks"])
                    self.assertIsNone(summary["github"]["current_clone_window"]["unique_cloners"])
                self.assertIn("unavailable", (output / "README.md").read_text())
                ET.fromstring((output / "usage.svg").read_text())

    def test_both_failed_sources_leave_existing_artifacts_unchanged(self):
        """A total outage attempts both sources and does not overwrite history."""
        before = {path.name: path.read_bytes() for path in self.output.iterdir()}
        with patch.object(METRICS, "fetch_pypi_history", side_effect=METRICS.MetricsError("offline")) as pypi, \
                patch.object(METRICS, "fetch_github_metrics", side_effect=METRICS.MetricsError("offline")) as github:
            with self.assertRaisesRegex(METRICS.MetricsError, "No sources collected"):
                METRICS.collect_metrics(self.output, "bsk", "AVSLab/basilisk", "fixture")
            pypi.assert_called_once()
            github.assert_called_once()
        self.assertEqual(before, {path.name: path.read_bytes() for path in self.output.iterdir()})

    def test_missing_github_token_still_collects_pypi(self):
        """A missing secret is reported per source without preventing PyPI collection."""
        with patch.object(METRICS, "fetch_pypi_history", return_value=self.pypi_rows), \
                patch.object(METRICS, "_request_json") as github_request:
            summary = METRICS.collect_metrics(self.output, "bsk", "AVSLab/basilisk", "")
        github_request.assert_not_called()
        self.assertEqual(summary["sources"]["pypi"]["status"], "ok")
        self.assertEqual(summary["sources"]["github"]["status"], "error")
        self.assertIn("token", summary["sources"]["github"]["error"])

    def test_recovery_refreshes_counts_and_clears_errors(self):
        """A recovered source replaces overlapping counts and becomes current again."""
        self.collect(pypi_error=METRICS.MetricsError("offline"))
        self.pypi_rows.append({"date": self.yesterday, "pypi_downloads": 80, "pypi_pip_downloads": 30})
        summary = self.collect()
        self.assertEqual(summary["collection_status"], "complete")
        self.assertEqual(summary["pypi"]["downloads_excluding_known_mirrors"], 120)
        self.assertIsNone(summary["sources"]["pypi"]["error"])
        rows = METRICS.load_history(self.output / "metrics.csv")
        self.assertEqual(len(rows), 2)

    def test_legacy_summary_retains_original_freshness_and_counting_policy(self):
        """Schema-one history is not misrepresented as a successful filtered refresh."""
        legacy = dict(self.previous, schema_version=1)
        del legacy["sources"]
        del legacy["pypi"]["counting_policy"]
        (self.output / "summary.json").write_text(json.dumps(legacy), encoding="utf-8")
        summary = self.collect(pypi_error=METRICS.MetricsError("offline"))
        self.assertEqual(summary["sources"]["pypi"]["last_success_at"], self.old_time)
        self.assertEqual(summary["pypi"]["counting_policy"], "legacy_unfiltered")


@unittest.skipUnless(os.name != "nt" and shutil.which("bash") and shutil.which("git"),
                     "The workflow restore script runs on POSIX GitHub runners")
class RestoreHistoryTest(unittest.TestCase):
    def setUp(self):
        """Create an isolated bare remote and a checkout with fetched references."""
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.remote = self.root / "remote.git"
        self.checkout = self.root / "checkout"
        # Hooks may inherit GIT_INDEX_FILE or GIT_DIR from the developer's
        # checkout. Do not let the fixture commands touch that repository.
        self.env = {
            **{key: value for key, value in os.environ.items() if not key.startswith("GIT_")},
            "GIT_CONFIG_GLOBAL": os.devnull, "GIT_CONFIG_NOSYSTEM": "1",
            "GIT_AUTHOR_NAME": "Test", "GIT_AUTHOR_EMAIL": "test@example.invalid",
            "GIT_COMMITTER_NAME": "Test", "GIT_COMMITTER_EMAIL": "test@example.invalid",
        }
        self.git("init", "--bare", str(self.remote), cwd=self.root)
        self.seed_branch("develop", {"README.md": "fixture"})
        self.git("clone", "--branch", "develop", str(self.remote), str(self.checkout), cwd=self.root)
        self.output = self.root / "output"
        self.script = SCRIPT_PATH.with_name("restore_usage_metrics.sh")

    def git(self, *args, cwd=None, data=None):
        return subprocess.run(["git", *args], cwd=cwd or self.remote, env=self.env,
                              input=data, text=True, capture_output=True, check=True).stdout.strip()

    def seed_branch(self, branch, files):
        entries = []
        for name, content in sorted(files.items()):
            blob = self.git("hash-object", "-w", "--stdin", data=content)
            entries.append(f"100644 blob {blob}\t{name}\n")
        tree = self.git("mktree", data="".join(entries))
        commit = self.git("commit-tree", tree, "-m", "Fixture")
        self.git("update-ref", "refs/heads/" + branch, commit)
        return commit

    def restore(self, failed_command=None):
        fault = ""
        if failed_command:
            fault = f'git() {{ if [[ "$1" == {failed_command} ]]; then return 128; fi; command git "$@"; }}\n'
        return subprocess.run(["bash", "-c", fault + 'source "$1" "$2"', "restore",
                               str(self.script), str(self.output)], cwd=self.checkout,
                              env=self.env, text=True, capture_output=True)

    def test_first_run_requires_a_successful_remote_lookup(self):
        """A confirmed absent branch allows collection; a lookup failure does not."""
        self.assertEqual(self.restore().returncode, 0)
        self.assertFalse((self.output / "metrics.csv").exists())
        self.assertNotEqual(self.restore("ls-remote").returncode, 0)

    def test_existing_history_and_freshness_are_restored(self):
        """The CSV and summary are restored byte for byte from the metrics branch."""
        files = {"metrics.csv": "old history\n", "summary.json": '{"generated_at":"old"}\n'}
        self.seed_branch("usage-metrics", files)
        result = self.restore()
        self.assertEqual(result.returncode, 0, result.stderr)
        for name, content in files.items():
            self.assertEqual((self.output / name).read_text(), content)

    def test_failed_fetch_stops_even_with_an_existing_remote_reference(self):
        """The reviewed history-loss scenario fails before collection can proceed."""
        commit = self.seed_branch("usage-metrics", {"metrics.csv": "old", "summary.json": "{}"})
        self.git("fetch", "origin", cwd=self.checkout)
        self.assertNotEqual(self.restore("fetch").returncode, 0)
        self.assertFalse((self.output / "metrics.csv").exists())
        self.assertEqual(self.git("rev-parse", "refs/heads/usage-metrics"), commit)

    def test_missing_history_file_stops_restoration(self):
        """An existing metrics branch must contain both durable artifacts."""
        for missing in ("metrics.csv", "summary.json"):
            with self.subTest(missing=missing):
                self.seed_branch("usage-metrics", {
                    name: "fixture" for name in ("metrics.csv", "summary.json") if name != missing
                })
                # Ensure subsequent fixtures do not cause a non-fast-forward
                # fetch failure before the missing-file check is exercised.
                self.git("fetch", "--force", "origin", cwd=self.checkout)
                self.assertNotEqual(self.restore().returncode, 0)

    def test_disappearing_remote_branch_does_not_discard_known_history(self):
        """A branch removed after checkout cannot be mistaken for initial setup."""
        self.seed_branch("usage-metrics", {"metrics.csv": "old", "summary.json": "{}"})
        self.git("fetch", "origin", cwd=self.checkout)
        self.git("update-ref", "-d", "refs/heads/usage-metrics")
        self.assertNotEqual(self.restore().returncode, 0)


if __name__ == "__main__":
    unittest.main()
