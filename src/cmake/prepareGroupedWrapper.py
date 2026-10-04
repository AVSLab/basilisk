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

"""Share SWIG's director mutex across wrappers without changing director RTTI."""

from pathlib import Path
import sys


def prepare_wrapper(source: Path, destination: Path) -> None:
    """Make the generated director mutex an inline C++17 variable.

    SWIG emits one external mutex definition per director wrapper. A combined
    library needs a single shared definition while retaining the common director
    class for cross-module casts and Python callbacks.

    :param source: Original SWIG-generated C++ wrapper.
    :param destination: C++ wrapper compiled by the combined target.
    :raises ValueError: If a director wrapper uses an unrecognized mutex definition.
    """
    content = source.read_text(encoding="utf-8")
    if "#define SWIG_DIRECTORS" in content:
        definition = "SWIG_GUARD_DEFINITION(Director, swig_mutex_own);"
        if content.count(definition) != 1:
            raise ValueError(f"Unrecognized SWIG director mutex in {source}")
        # The declaration macro can be empty when threading is disabled.
        replacement = "#ifdef SWIG_THREADS\n  inline " + definition + "\n#endif"
        content = content.replace(definition, replacement)
    destination.write_text(content, encoding="utf-8")


if __name__ == "__main__":
    prepare_wrapper(*(Path(arg) for arg in sys.argv[1:3]))
