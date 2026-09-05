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

"""Collect durable PyPI and GitHub usage metrics for Basilisk."""

import argparse
import csv
import html
import json
import math
import os
import re
import sys
import time
from datetime import date, datetime, timedelta, timezone
from http.client import HTTPException
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple
from urllib.error import HTTPError
from urllib.parse import urlencode
from urllib.request import Request, urlopen


CLICKPY_URL = "https://sql-clickhouse.clickhouse.com/"
GITHUB_API_URL = "https://api.github.com"
REQUEST_TIMEOUT_SECONDS = 120  # [s]
RETRY_DELAY_SECONDS = 5  # [s]
REQUEST_ATTEMPTS = 3
ROLLING_WINDOW_DAYS = 7  # [day]
CONTIGUOUS_DAY_STEP = 1  # [day]
CSV_FIELDS = (
    "date",
    "pypi_downloads",
    "pypi_pip_downloads",
    "github_clones",
    "github_unique_cloners",
    "github_forks",
    "github_release_asset_downloads",
)
PYPI_MIRROR_INSTALLERS = (
    "bandersnatch",
    "z3c.pypimirror",
    "artifactory",
    "devpi",
)


class MetricsError(RuntimeError):
    """Report a failure to retrieve or validate usage metrics."""


def _request_text(
    url: str,
    *,
    data: Optional[bytes] = None,
    headers: Optional[Dict[str, str]] = None,
) -> str:
    request_headers = {
        "Accept": "application/json",
        "User-Agent": "basilisk-usage-metrics/1",
    }
    if headers:
        request_headers.update(headers)

    for attempt in range(REQUEST_ATTEMPTS):
        request = Request(url, data=data, headers=request_headers)
        try:
            with urlopen(request, timeout=REQUEST_TIMEOUT_SECONDS) as response:
                return response.read().decode("utf-8")
        except HTTPError as error:
            retryable = error.code == 429 or 500 <= error.code < 600
            if not retryable or attempt == REQUEST_ATTEMPTS - 1:
                raise MetricsError(f"Request to {url} failed: HTTP {error.code}") from error
        except (HTTPException, OSError) as error:
            # Body reads can raise these directly, without urllib wrapping
            # them in URLError (which, like TimeoutError, is an OSError).
            if attempt == REQUEST_ATTEMPTS - 1:
                raise MetricsError(f"Request to {url} failed: {error}") from error

        time.sleep(RETRY_DELAY_SECONDS*(attempt+1))

    raise MetricsError(f"Request to {url} failed")


def _request_json(url: str, *, token: Optional[str] = None) -> Any:
    headers = {"X-GitHub-Api-Version": "2022-11-28"}
    if token:
        headers["Authorization"] = f"Bearer {token}"
    response = _request_text(url, headers=headers)
    try:
        return json.loads(response)
    except json.JSONDecodeError as error:
        raise MetricsError(f"Request to {url} returned invalid JSON") from error


def fetch_pypi_history(package_name: str) -> List[Dict[str, Any]]:
    """Return daily counts excluding identifiable non-distribution files.

    Older records without filenames are retained. An absent date remains
    unknown because the source may have an ingestion gap.
    """
    if not re.fullmatch(r"[A-Za-z0-9._-]+", package_name):
        raise MetricsError(f"Invalid PyPI package name: {package_name}")

    mirror_names = ", ".join(f"'{name}'" for name in PYPI_MIRROR_INSTALLERS)
    query = f"""
        WITH (
            filename = '' OR endsWith(filename, '.whl')
            OR endsWith(filename, '.tar.gz') OR endsWith(filename, '.zip')
        ) AS distribution_or_unknown
        SELECT
            date,
            countIf(distribution_or_unknown AND lower(installer) NOT IN ({mirror_names})) AS downloads,
            countIf(distribution_or_unknown AND lower(installer) = 'pip') AS pip_downloads
        FROM pypi.pypi
        WHERE project = {{package:String}}
        GROUP BY date
        ORDER BY date
        FORMAT JSONEachRow
    """
    query_parameters = urlencode({"user": "demo", "param_package": package_name})
    response = _request_text(
        f"{CLICKPY_URL}?{query_parameters}",
        data=query.encode("utf-8"),
        headers={"Content-Type": "text/plain; charset=utf-8"},
    )

    rows = []
    try:
        for line in response.splitlines():
            if not line.strip():
                continue
            raw_row = json.loads(line)
            rows.append(
                {
                    "date": date.fromisoformat(raw_row["date"]).isoformat(),
                    "pypi_downloads": int(raw_row["downloads"]),
                    "pypi_pip_downloads": int(raw_row["pip_downloads"]),
                }
            )
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise MetricsError("ClickPy returned an unexpected response") from error

    if not rows:
        raise MetricsError(f"ClickPy returned no data for {package_name}")
    return rows


def _github_url(repository: str, endpoint: str, **query: Any) -> str:
    url = f"{GITHUB_API_URL}/repos/{repository}/{endpoint}"
    if query:
        url = f"{url}?{urlencode(query)}"
    return url


def fetch_github_metrics(repository: str, token: str) -> Dict[str, Any]:
    """Return current repository, clone, and release-asset metrics."""
    if not re.fullmatch(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+", repository):
        raise MetricsError(f"Invalid GitHub repository: {repository}")
    if not token:
        raise MetricsError("A GitHub token with repository Administration read permission is required")

    repository_data = _request_json(f"{GITHUB_API_URL}/repos/{repository}", token=token)
    clone_data = _request_json(
        _github_url(repository, "traffic/clones", per="day"),
        token=token,
    )

    release_asset_downloads = 0
    page = 1
    while True:
        releases = _request_json(
            _github_url(repository, "releases", per_page=100, page=page),
            token=token,
        )
        if not isinstance(releases, list):
            raise MetricsError("GitHub returned an unexpected releases response")
        try:
            for release in releases:
                for asset in release.get("assets", []):
                    release_asset_downloads += int(asset["download_count"])
        except (AttributeError, KeyError, TypeError, ValueError) as error:
            raise MetricsError("GitHub returned an unexpected release asset") from error
        if len(releases) < 100:
            break
        page += 1

    try:
        clones = [
            {
                "date": item["timestamp"][:10],
                "github_clones": int(item["count"]),
                "github_unique_cloners": int(item["uniques"]),
            }
            for item in clone_data["clones"]
        ]
        return {
            "clones": clones,
            "clone_window_count": int(clone_data["count"]),
            "clone_window_unique_cloners": int(clone_data["uniques"]),
            "forks": int(repository_data["forks_count"]),
            "release_asset_downloads": release_asset_downloads,
        }
    except (KeyError, TypeError, ValueError) as error:
        raise MetricsError("GitHub returned an unexpected metrics response") from error


def _empty_history_row(date: str) -> Dict[str, Any]:
    row = {field: None for field in CSV_FIELDS}
    row["date"] = date
    return row


def load_history(path: Path) -> List[Dict[str, Any]]:
    """Load the previously collected CSV history, if present."""
    if not path.exists():
        return []

    rows = []
    with path.open(newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        if tuple(reader.fieldnames or ()) != CSV_FIELDS:
            raise MetricsError(f"Unexpected columns in {path}")
        for raw_row in reader:
            row = _empty_history_row(raw_row["date"])
            for field in CSV_FIELDS[1:]:
                value = raw_row[field]
                row[field] = int(value) if value else None
            rows.append(row)
    return rows


def merge_history(
    existing_rows: Iterable[Dict[str, Any]],
    pypi_rows: Iterable[Dict[str, Any]],
    github_metrics: Optional[Dict[str, Any]],
    snapshot_date: str,
) -> List[Dict[str, Any]]:
    """Merge overlapping source windows into one row per UTC date."""
    rows_by_date = {
        row["date"]: _empty_history_row(row["date"]) | dict(row)
        for row in existing_rows
    }

    for source_row in pypi_rows:
        row = rows_by_date.setdefault(source_row["date"], _empty_history_row(source_row["date"]))
        row["pypi_downloads"] = source_row["pypi_downloads"]
        row["pypi_pip_downloads"] = source_row["pypi_pip_downloads"]

    if github_metrics is not None:
        for source_row in github_metrics["clones"]:
            row = rows_by_date.setdefault(source_row["date"], _empty_history_row(source_row["date"]))
            row["github_clones"] = source_row["github_clones"]
            row["github_unique_cloners"] = source_row["github_unique_cloners"]

        snapshot_row = rows_by_date.setdefault(snapshot_date, _empty_history_row(snapshot_date))
        snapshot_row["github_forks"] = github_metrics["forks"]
        snapshot_row["github_release_asset_downloads"] = github_metrics["release_asset_downloads"]

    if rows_by_date:
        first_day = date.fromisoformat(min(rows_by_date))
        last_day = date.fromisoformat(max(rows_by_date))
        for offset in range((last_day - first_day).days + 1):
            day = (first_day + timedelta(days=offset)).isoformat()
            rows_by_date.setdefault(day, _empty_history_row(day))
    return [rows_by_date[date] for date in sorted(rows_by_date)]


def write_history(path: Path, rows: Iterable[Dict[str, Any]]) -> None:
    """Write history in a stable CSV format."""
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    field: "" if row.get(field) is None else row[field]
                    for field in CSV_FIELDS
                }
            )


def _coverage(rows: List[Dict[str, Any]], field: str) -> Dict[str, Optional[str]]:
    dates = [row["date"] for row in rows if row.get(field) is not None]
    return {
        "start": min(dates) if dates else None,
        "end": max(dates) if dates else None,
    }


def build_summary(
    rows: List[Dict[str, Any]],
    package_name: str,
    repository: str,
    github_metrics: Optional[Dict[str, Any]],
    generated_at: str,
    sources: Optional[Dict[str, Any]] = None,
    previous_summary: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Build the machine-readable current summary."""
    clone_coverage = _coverage(rows, "github_clones")
    pypi_coverage = _coverage(rows, "pypi_downloads")
    previous_github = (previous_summary or {}).get("github", {})
    clone_window = previous_github.get("current_clone_window", {
        "start": None, "end": None, "clones": None, "unique_cloners": None,
    })
    if github_metrics is not None:
        clone_dates = [item["date"] for item in github_metrics["clones"]]
        clone_window = {
            "start": min(clone_dates) if clone_dates else None,
            "end": max(clone_dates) if clone_dates else None,
            "clones": github_metrics["clone_window_count"],
            "unique_cloners": github_metrics["clone_window_unique_cloners"],
        }

    def latest_value(field: str) -> Optional[int]:
        return next((row[field] for row in reversed(rows) if row.get(field) is not None), None)

    return {
        "schema_version": 2,
        "generated_at": generated_at,
        "collection_status": "partial" if any(
            source["status"] == "error" for source in (sources or {}).values()
        ) else "complete",
        "sources": sources or {},
        "pypi": {
            "package": package_name,
            "counting_policy": (
                (previous_summary or {}).get("pypi", {}).get("counting_policy", "legacy_unfiltered")
                if (sources or {}).get("pypi", {}).get("status") == "error"
                else "distribution_files_or_unknown_filename"
            ),
            "coverage": pypi_coverage,
            "downloads_excluding_known_mirrors": sum(
                row["pypi_downloads"] or 0 for row in rows
            ) if pypi_coverage["start"] is not None else None,
            "pip_downloads": sum(row["pypi_pip_downloads"] or 0 for row in rows)
            if pypi_coverage["start"] is not None else None,
        },
        "github": {
            "repository": repository,
            "tracked_clone_coverage": clone_coverage,
            "tracked_clones": sum(row["github_clones"] or 0 for row in rows)
            if clone_coverage["start"] is not None else None,
            "current_clone_window": clone_window,
            "forks": latest_value("github_forks"),
            "release_asset_downloads": latest_value("github_release_asset_downloads"),
        },
    }


def _rolling_series(
    rows: List[Dict[str, Any]],
    field: str,
) -> List[Tuple[int, float]]:
    """Return means only for seven consecutive observed days, including zeros."""
    window: List[Tuple[date, int]] = []
    points = []
    for row in rows:
        value = row.get(field)
        if value is None:
            window = []
            continue
        current_date = date.fromisoformat(row["date"])
        if window and (current_date - window[-1][0]).days != CONTIGUOUS_DAY_STEP:
            window = []
        window.append((current_date, value))
        window = window[-ROLLING_WINDOW_DAYS:]
        if len(window) == ROLLING_WINDOW_DAYS:
            points.append(
                (
                    current_date.toordinal(),
                    sum(item[1] for item in window) / ROLLING_WINDOW_DAYS,
                )
            )
    return points


def _nice_ceiling(maximum: float) -> float:
    """Round a positive chart maximum to a readable upper bound."""
    if maximum <= 0:
        return 1.0
    magnitude = 10**math.floor(math.log10(maximum))
    normalized = maximum/magnitude
    for multiplier in (1, 2, 5, 10):
        if normalized <= multiplier:
            return multiplier*magnitude
    return 10*magnitude


def _series_path(
    points: List[Tuple[int, float]],
    x_minimum: int,
    x_maximum: int,
    y_maximum: float,
    plot_bounds: Tuple[float, float, float, float],
) -> str:
    """Convert dated values to an SVG path while retaining data gaps."""
    left, top, width, height = plot_bounds
    x_span = max(x_maximum - x_minimum, 1)
    commands = []
    previous_day = None
    for day, value in points:
        x_position = left + width*(day - x_minimum)/x_span
        y_position = top + height*(1 - value/y_maximum)
        command = "L"
        if previous_day is None or day - previous_day > CONTIGUOUS_DAY_STEP:
            command = "M"
        commands.append(f"{command}{x_position:.1f},{y_position:.1f}")
        if command == "M":
            commands.append("l0,0")
        previous_day = day
    return " ".join(commands)


def _render_chart_panel(
    rows: List[Dict[str, Any]],
    series: List[Tuple[str, str, str]],
    title: str,
    y_axis_title: str,
    plot_top: float,
) -> List[str]:
    """Render one Cartesian SVG panel."""
    plot_left = 90.0
    plot_width = 875.0
    plot_height = 225.0
    plot_bounds = (plot_left, plot_top, plot_width, plot_height)
    series_points = {
        field: _rolling_series(rows, field)
        for field, _, _ in series
    }
    all_points = [point for points in series_points.values() for point in points]
    if not all_points:
        return [
            f'<text class="panel-title" x="{plot_left}" y="{plot_top - 24}">{title}</text>',
            (
                f'<text class="axis-text" x="{plot_left + plot_width/2}" '
                f'y="{plot_top + plot_height/2}" text-anchor="middle">'
                "No complete seven-day window available</text>"
            ),
        ]

    x_minimum = min(point[0] for point in all_points)
    x_maximum = max(point[0] for point in all_points)
    y_maximum = _nice_ceiling(max(point[1] for point in all_points))
    elements = [
        f'<text class="panel-title" x="{plot_left}" y="{plot_top - 24}">{title}</text>',
        (
            f'<rect class="chart-frame" x="{plot_left}" y="{plot_top}" '
            f'width="{plot_width}" height="{plot_height}" />'
        ),
    ]

    tick_count = 4
    for tick_index in range(tick_count + 1):
        tick_value = y_maximum*tick_index/tick_count
        y_position = plot_top + plot_height*(1 - tick_index/tick_count)
        elements.extend(
            [
                (
                    f'<line class="grid-line" x1="{plot_left}" y1="{y_position:.1f}" '
                    f'x2="{plot_left + plot_width}" y2="{y_position:.1f}" />'
                ),
                (
                    f'<text class="axis-text" x="{plot_left - 10}" y="{y_position + 4:.1f}" '
                    f'text-anchor="end">{tick_value:,.0f}</text>'
                ),
            ]
        )

    x_tick_count = 5
    x_span = max(x_maximum - x_minimum, 1)
    x_tick_days = sorted(
        {
            round(x_minimum + x_span*tick_index/(x_tick_count - 1))
            for tick_index in range(x_tick_count)
        }
    )
    for tick_day in x_tick_days:
        x_position = plot_left + plot_width*(tick_day - x_minimum)/x_span
        tick_label = date.fromordinal(tick_day).isoformat()
        elements.extend(
            [
                (
                    f'<line class="axis-tick" x1="{x_position:.1f}" '
                    f'y1="{plot_top + plot_height}" x2="{x_position:.1f}" '
                    f'y2="{plot_top + plot_height + 6}" />'
                ),
                (
                    f'<text class="axis-text" x="{x_position:.1f}" '
                    f'y="{plot_top + plot_height + 23}" '
                    f'text-anchor="middle">{tick_label}</text>'
                ),
            ]
        )

    elements.extend(
        [
            (
                f'<text class="axis-title" x="{plot_left + plot_width/2}" '
                f'y="{plot_top + plot_height + 45}" text-anchor="middle">UTC date</text>'
            ),
            (
                f'<text class="axis-title" x="22" '
                f'y="{plot_top + plot_height/2}" text-anchor="middle" '
                f'transform="rotate(-90 22 {plot_top + plot_height/2})">'
                f'{y_axis_title}</text>'
            ),
        ]
    )

    legend_x = plot_left + plot_width - 385
    legend_y = plot_top - 29
    for series_index, (field, label, css_class) in enumerate(series):
        item_x = legend_x + 190*series_index
        path_data = _series_path(
            series_points[field],
            x_minimum,
            x_maximum,
            y_maximum,
            plot_bounds,
        )
        elements.extend(
            [
                (
                    f'<line class="legend-line {css_class}" x1="{item_x}" '
                    f'y1="{legend_y}" x2="{item_x + 28}" y2="{legend_y}" />'
                ),
                (
                    f'<text class="legend-text" x="{item_x + 36}" '
                    f'y="{legend_y + 4}">{label}</text>'
                ),
                (
                    f'<path class="data-line {css_class}" '
                    f'd="{path_data}" />'
                ),
            ]
        )
    return elements


def render_usage_svg(
    rows: List[Dict[str, Any]],
    generated_at: str,
    sources: Optional[Dict[str, Any]] = None,
) -> str:
    """Render the retained usage history as a self-contained SVG chart."""
    elements = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        (
            '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1000 760" '
            'role="img" aria-labelledby="chart-title chart-description">'
        ),
        '<title id="chart-title">Basilisk package and repository usage</title>',
        (
            '<desc id="chart-description">Seven-day trailing means of daily PyPI '
            'downloads and GitHub clone activity. Only complete windows of seven '
            'observed days are shown.</desc>'
        ),
        """<style>
            .background { fill: #ffffff; }
            .chart-frame { fill: none; stroke: #8c959f; stroke-width: 1; }
            .grid-line { stroke: #d8dee4; stroke-width: 1; }
            .axis-tick { stroke: #57606a; stroke-width: 1; }
            .chart-title, .panel-title, .axis-title, .axis-text, .legend-text {
                fill: #24292f;
                font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
            }
            .chart-title { font-size: 22px; font-weight: 600; }
            .chart-subtitle { fill: #57606a; font-size: 12px; }
            .panel-title { font-size: 16px; font-weight: 600; }
            .axis-title { font-size: 12px; font-weight: 600; }
            .axis-text, .legend-text { font-size: 11px; }
            .data-line, .legend-line {
                fill: none;
                stroke-linejoin: round;
                stroke-linecap: round;
                stroke-width: 2.5;
                vector-effect: non-scaling-stroke;
            }
            .pypi-downloads { stroke: #0969da; }
            .pip-downloads { stroke: #bf8700; }
            .github-clones { stroke: #1a7f37; }
            .github-unique { stroke: #8250df; }
            @media (prefers-color-scheme: dark) {
                .background { fill: #0d1117; }
                .chart-frame { stroke: #6e7681; }
                .grid-line { stroke: #30363d; }
                .axis-tick { stroke: #8b949e; }
                .chart-title, .panel-title, .axis-title, .axis-text, .legend-text {
                    fill: #e6edf3;
                }
                .chart-subtitle { fill: #8b949e; }
                .pypi-downloads { stroke: #58a6ff; }
                .pip-downloads { stroke: #d29922; }
                .github-clones { stroke: #3fb950; }
                .github-unique { stroke: #bc8cff; }
            }
        </style>""",
        '<rect class="background" width="1000" height="760" />',
        '<text id="chart-title-text" class="chart-title" x="30" y="34">Basilisk usage</text>',
        (
            '<text class="chart-title chart-subtitle" x="970" y="32" '
            f'text-anchor="end">Updated {html.escape(generated_at)}</text>'
        ),
    ]
    elements.extend(
        _render_chart_panel(
            rows,
            [
                ("pypi_downloads", "Non-mirror", "pypi-downloads"),
                ("pypi_pip_downloads", "pip", "pip-downloads"),
            ],
            "Daily PyPI download events — 7-day trailing mean",
            "Downloads/day",
            90.0,
        )
    )
    elements.extend(
        _render_chart_panel(
            rows,
            [
                ("github_clones", "Clones", "github-clones"),
                ("github_unique_cloners", "Unique cloners", "github-unique"),
            ],
            "Daily GitHub clone activity — 7-day trailing mean",
            "Events/day",
            425.0,
        )
    )
    elements.append(
        '<text class="axis-text" x="30" y="723">'
        'PyPI: known non-distribution files excluded after a successful refresh; '
        'older unnamed files remain.</text>'
    )
    freshness = "; ".join(
        f"{name}: {source['status']}; last success {source['last_success_at'] or 'never'}"
        for name, source in (sources or {}).items()
    )
    elements.extend([
        f'<text class="axis-text" x="30" y="745">{html.escape(freshness)}</text>',
        "</svg>",
    ])
    return "\n".join(elements) + "\n"


def render_readme(summary: Dict[str, Any]) -> str:
    """Render a human-readable landing page for the metrics branch."""
    pypi = summary["pypi"]
    github = summary["github"]
    clone_window = github["current_clone_window"]
    pypi_coverage = f"{pypi['coverage']['start']} through {pypi['coverage']['end']}"
    tracked_clone_coverage = (
        f"{github['tracked_clone_coverage']['start']} through "
        f"{github['tracked_clone_coverage']['end']}"
    )
    clone_window_coverage = f"{clone_window['start']} through {clone_window['end']}"

    def count(value: Optional[int]) -> str:
        return f"{value:,}" if value is not None else "unavailable"

    lines = [
        "# Basilisk usage metrics",
        "",
        f"Updated {summary['generated_at']}.",
        f"Collection status: **{summary['collection_status']}**.",
        "",
        "| Source | Status | Last successful collection (UTC) | Latest error |",
        "|---|---|---|---|",
    ]
    for name, source in summary["sources"].items():
        error = (source["error"] or "none").replace("|", "\\|").replace("\n", " ")
        lines.append(
            f"| {name} | {source['status']} | {source['last_success_at'] or 'never'} | {error} |"
        )
    lines.extend([
        "",
        "Failed sources retain their previous observations; they are not new daily snapshots.",
        "Collection time does not guarantee that the upstream dataset is current; see coverage dates.",
        "",
        "| Metric | Count | Coverage |",
        "|---|---:|---|",
        (
            "| PyPI downloads excluding known mirrors | "
            f"{count(pypi['downloads_excluding_known_mirrors'])} | {pypi_coverage} |"
        ),
        f"| PyPI downloads made by `pip` | {count(pypi['pip_downloads'])} | {pypi_coverage} |",
        (
            "| GitHub clone events retained by this tracker | "
            f"{count(github['tracked_clones'])} | {tracked_clone_coverage} |"
        ),
        (
            "| GitHub clones in the last successful API window | "
            f"{count(clone_window['clones'])} | {clone_window_coverage} |"
        ),
        (
            "| Unique GitHub cloners in the last successful API window | "
            f"{count(clone_window['unique_cloners'])} | {clone_window_coverage} |"
        ),
        f"| Last observed GitHub forks | {count(github['forks'])} | snapshot |",
        (
            "| GitHub release-asset downloads | "
            f"{count(github['release_asset_downloads'])} | cumulative snapshot |"
        ),
        "",
        "![Daily Basilisk PyPI download and GitHub clone activity](usage.svg)",
        "",
        "`metrics.csv` contains the daily history and `summary.json` contains the latest",
        "machine-readable totals. `usage.svg` contains the plotted seven-day trends.",
        "",
        "## Interpretation",
        "",
        "- PyPI counts are file-download events, not verified installations or unique users.",
        "  The `pip` count selects records whose installer is identified as `pip`.",
        "- The non-mirror count excludes `bandersnatch`, `z3c.pypimirror`, `Artifactory`,",
        "  and `devpi`, matching the known-mirror list documented by PyPI Stats.",
        "- Counts exclude identified non-distribution files, including `.metadata` sidecars.",
        "  Older records without filenames remain and may include metadata requests.",
        "  PyPI changed its logging on 2026-08-24; historical counts are not fully comparable.",
        f"  Retained PyPI counting policy: `{pypi['counting_policy']}`.",
        "  A successful PyPI refresh applies the revised filter to returned historical dates.",
        "- Plots require seven consecutive observed days. Explicit zeros count; absent dates",
        "  remain unknown and break the plotted series. The latest reported day may be partial.",
        "- GitHub exposes clone traffic for only the latest 14 days. The tracker preserves",
        "  those daily counts; gaps longer than 14 days cannot be recovered.",
        "- Daily unique-cloner values cannot be summed into an all-time unique-user count.",
        "- Forks are currently existing forks, not a lifetime count of every fork ever made.",
        "- Release downloads cover uploaded release assets only. GitHub provides no download",
        "  count for its automatically generated source ZIP and tar archives.",
        "",
        "The data comes from the public PyPI dataset through",
        "[ClickPy](https://clickpy.clickhouse.com/) and from the",
        "[GitHub REST API](https://docs.github.com/en/rest/metrics/traffic).",
    ])
    return "\n".join(lines) + "\n"


def collect_metrics(
    output_directory: Path,
    package_name: str,
    repository: str,
    github_token: str,
) -> Dict[str, Any]:
    """Collect, merge, and write all usage-metric artifacts."""
    output_directory.mkdir(parents=True, exist_ok=True)
    history_path = output_directory / "metrics.csv"
    existing_rows = load_history(history_path)
    summary_path = output_directory / "summary.json"
    previous_summary = {}
    if summary_path.exists():
        try:
            previous_summary = json.loads(summary_path.read_text(encoding="utf-8"))
            if not isinstance(previous_summary, dict) or previous_summary.get("schema_version") not in (1, 2):
                raise ValueError("Unknown summary schema")
            for name in ("pypi", "github"):
                if not isinstance(previous_summary.get(name), dict):
                    raise ValueError(f"Missing {name} summary")
            if previous_summary["pypi"].get("package") != package_name or previous_summary["github"].get(
                "repository"
            ) != repository:
                raise ValueError("History belongs to a different package or repository")
        except (ValueError, TypeError) as error:
            raise MetricsError(f"Invalid previous summary in {summary_path}: {error}") from error

    errors = {}
    github_metrics = None
    pypi_rows = []
    try:
        github_metrics = fetch_github_metrics(repository, github_token)
    except MetricsError as error:
        errors["github"] = str(error)
    try:
        pypi_rows = fetch_pypi_history(package_name)
    except MetricsError as error:
        errors["pypi"] = str(error)
    if len(errors) == 2:
        raise MetricsError("No sources collected: " + "; ".join(f"{name}: {error}" for name, error in errors.items()))

    generated_at = datetime.now(timezone.utc).replace(microsecond=0).isoformat()
    sources = {}
    for name in ("pypi", "github"):
        previous_source = previous_summary.get("sources", {}).get(name, {})
        last_success = previous_source.get("last_success_at")
        if previous_summary.get("schema_version") == 1:
            last_success = previous_summary.get("generated_at")
        sources[name] = {
            "status": "error" if name in errors else "ok",
            "last_attempt_at": generated_at,
            "last_success_at": last_success if name in errors else generated_at,
            "error": errors.get(name),
        }
    snapshot_date = generated_at[:10]
    rows = merge_history(existing_rows, pypi_rows, github_metrics, snapshot_date)
    summary = build_summary(
        rows,
        package_name,
        repository,
        github_metrics,
        generated_at,
        sources,
        previous_summary,
    )

    write_history(history_path, rows)
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (output_directory / "README.md").write_text(
        render_readme(summary),
        encoding="utf-8",
    )
    (output_directory / "usage.svg").write_text(
        render_usage_svg(rows, generated_at, sources),
        encoding="utf-8",
    )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--package", default="bsk")
    parser.add_argument("--repository", default="AVSLab/basilisk")
    parser.add_argument("--github-token-env", default="BSK_TRAFFIC_TOKEN")
    args = parser.parse_args()

    github_token = os.environ.get(args.github_token_env, "")

    try:
        summary = collect_metrics(
            args.output_dir,
            args.package,
            args.repository,
            github_token,
        )
    except MetricsError as error:
        raise SystemExit(str(error)) from error

    print(json.dumps(summary, indent=2, sort_keys=True))
    if summary["collection_status"] == "partial":
        print("Wrote artifacts with partial source data; inspect summary.json for errors.", file=sys.stderr)


if __name__ == "__main__":
    main()
