#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

import subprocess
import sys

import pytest

from Basilisk.architecture import bskLogging


def test_bsk_error_treats_python_message_as_text():
    """Raise ``BasiliskError`` without treating Python text as a format string."""
    bsk_logger = bskLogging.BSKLogger()

    with pytest.raises(bskLogging.BasiliskError, match="preformatted %s message"):
        bsk_logger.error("preformatted %s message")


def test_bsk_log_treats_python_message_as_text():
    """Preserve format directives passed through the Python ``bskLog`` API."""
    bsk_logger = bskLogging.BSKLogger()
    message = "preformatted %s message"

    with pytest.raises(bskLogging.BasiliskError) as error:
        bsk_logger.bskLog(bskLogging.BSK_ERROR, message)

    assert str(error.value) == message


def test_warning_level_output_is_flushed():
    """A ``BSK_WARNING`` must reach stdout at log time (issue #1444).

    Use an OS pipe because a Windows Debug extension has a separate C runtime
    from release Python, whose file descriptors ``capfd`` redirects. The child
    exits without normal C stream cleanup, so removing the warning flush loses
    the marker and fails the test.
    """
    result = subprocess.run(
        [sys.executable, "-c", """
import os
from Basilisk.architecture import bskLogging
logger = bskLogging.BSKLogger()
logger.setLogLevel(bskLogging.BSK_WARNING)
logger.bskLog(bskLogging.BSK_WARNING, "flush regression marker")
print("after native warning", flush=True)
os._exit(0)
"""],
        capture_output=True, text=True, check=True, timeout=60,  # [s]
    )
    assert "flush regression marker" in result.stdout
    assert result.stdout.index("flush regression marker") < result.stdout.index("after native warning")
