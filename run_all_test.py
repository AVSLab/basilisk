"""Run the Basilisk C/C++, Python, and available Rust test suites."""

import shutil
import subprocess
from pathlib import Path


def run_rust_tests(repository_root: Path) -> None:
    """Run the Cargo workspace tests when the Rust toolchain is available."""
    cargo = shutil.which("cargo")
    if cargo is None:
        print("Cargo was not found on PATH; skipping Rust tests.", flush=True)
        return

    print("Running Rust workspace tests.", flush=True)
    subprocess.run(
        [
            cargo,
            "test",
            "--workspace",
            "--all-features",
            "--locked",
            "--manifest-path",
            "src/Cargo.toml",
        ],
        cwd=repository_root,
        check=True,
    )


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
