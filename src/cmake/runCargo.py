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

"""Hide routine Cargo progress while preserving diagnostics and failure status."""

import argparse
import re
import subprocess
import sys


ANSI_SGR = re.compile(rb"\x1b\[[0-9;]*m")
CARGO_PROGRESS = re.compile(
    rb"(?: {3}Compiling [A-Za-z0-9_-]+ v[0-9][^\r\n]*"
    rb"| {4}Finished `[^`]+` profile \[[^]\r\n]+\] target\(s\) in [0-9][^\r\n]*)\r?\n"
)


def run_cargo(command, verbose=False):
    """Forward Cargo output verbatim, omitting only known progress before diagnostics.

    Once any other stderr text appears, forward the rest of that invocation
    unchanged. This preserves multiline diagnostics even if their source excerpts
    resemble progress messages. Stdout is always inherited without filtering.
    """
    try:
        if verbose:
            returncode = subprocess.call(command)
        else:
            with subprocess.Popen(command, stderr=subprocess.PIPE) as process:
                show_remaining = False
                for line in process.stderr:
                    plain = ANSI_SGR.sub(b"", line)
                    if not show_remaining and CARGO_PROGRESS.fullmatch(plain):
                        continue
                    sys.stderr.buffer.write(line)
                    sys.stderr.buffer.flush()
                    if plain.strip():
                        show_remaining = True
                returncode = process.wait()
    except OSError as error:
        print(f"error: could not start Cargo: {error}", file=sys.stderr)
        return 127
    except KeyboardInterrupt:
        return 130
    # Python represents signal termination with a negative return code. Use the
    # conventional shell status so the enclosing build still reports failure.
    return returncode if returncode >= 0 else 128 - returncode


def main():
    """Run the selected Cargo executable with its original argument list."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verbose", action="store_true", help="show all Cargo output")
    parser.add_argument("cargo", help="Cargo executable")
    parser.add_argument("arguments", nargs=argparse.REMAINDER)
    arguments = parser.parse_args()
    return run_cargo([arguments.cargo, *arguments.arguments], arguments.verbose)


if __name__ == "__main__":
    sys.exit(main())
