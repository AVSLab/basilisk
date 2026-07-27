"""Run the Basilisk C/C++, Python, and available Rust test suites."""

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

from colorama import Fore, Style, just_fix_windows_console


_RUST_TEST_SEPARATOR = "-" * 79
_RUST_TEST_RESULT_PATTERN = re.compile(
    r"^test result: (?:ok|FAILED)\. "
    r"(?P<passed>\d+) passed; "
    r"(?P<failed>\d+) failed; "
    r"(?P<ignored>\d+) ignored; "
    r"(?P<measured>\d+) measured; "
    r"(?P<filtered>\d+) filtered out;"
)
_USE_TERMINAL_COLORS = (
    sys.stdout.isatty()
    and "NO_COLOR" not in os.environ
    and os.environ.get("TERM") != "dumb"
)

if _USE_TERMINAL_COLORS:
    just_fix_windows_console()


def _color_status(text: str, color: str) -> str:
    """Return bright terminal-colored status text when color is supported."""
    if not _USE_TERMINAL_COLORS:
        return text
    return f"{Style.BRIGHT}{color}{text}{Style.RESET_ALL}"


def run_rust_tests(repository_root: Path) -> None:
    """Run and summarize Cargo tests when the Rust toolchain is available."""
    print(f"\n{_RUST_TEST_SEPARATOR}", flush=True)
    cargo = shutil.which("cargo")
    if cargo is None:
        print(
            _color_status(
                "Cargo was not found on PATH; skipping Rust tests.",
                Fore.YELLOW,
            ),
            flush=True,
        )
        return

    print(
        _color_status("Running Rust workspace tests.", Fore.GREEN),
        flush=True,
    )
    command = [
        cargo,
        "test",
        "--quiet",
        "--workspace",
        "--all-features",
        "--locked",
        "--manifest-path",
        "src/Cargo.toml",
        "--",
        "--format",
        "terse",
    ]
    totals = {
        "passed": 0,
        "failed": 0,
        "ignored": 0,
        "measured": 0,
        "filtered": 0,
    }
    result_count = 0
    process = subprocess.Popen(
        command,
        cwd=repository_root,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    assert process.stdout is not None
    for line in process.stdout:
        print(line, end="", flush=True)
        result = _RUST_TEST_RESULT_PATTERN.match(line)
        if result is None:
            continue
        result_count += 1
        for name in totals:
            totals[name] += int(result.group(name))

    return_code = process.wait()
    if result_count:
        test_count = sum(totals[name] for name in ("passed", "failed", "ignored", "measured"))
        status = "PASSED" if return_code == 0 else "FAILED"
        summary = (
            "Rust workspace tests: "
            f"{test_count} total; "
            f"{totals['passed']} passed; "
            f"{totals['failed']} failed; "
            f"{totals['ignored']} ignored; "
            f"{totals['filtered']} filtered out - {status}."
        )
        print(
            "\n" + _color_status(
                summary,
                Fore.GREEN if return_code == 0 else Fore.RED,
            ),
            flush=True,
        )
    elif return_code != 0:
        print(
            "\n"
            + _color_status(
                "Rust workspace tests failed before reporting test results.",
                Fore.RED,
            ),
            flush=True,
        )

    if return_code != 0:
        raise subprocess.CalledProcessError(return_code, command)


def main() -> None:
    """Run all test suites from the repository root."""
    repository_root = Path(__file__).resolve().parent
    subprocess.run(
        ["ctest", "-C", "Release"],
        cwd=repository_root / "dist3",
        check=True,
    )
    run_rust_tests(repository_root)
    subprocess.run(
        ["pytest", "-n", "auto"],
        cwd=repository_root / "src",
        check=True,
    )


if __name__ == "__main__":
    main()
