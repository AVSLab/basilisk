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

import pytest

from Basilisk.architecture import bskLogging


def test_bsk_error_treats_python_message_as_text():
    """Raise ``BasiliskError`` without treating Python text as a format string."""
    bsk_logger = bskLogging.BSKLogger()

    with pytest.raises(bskLogging.BasiliskError, match="preformatted %s message"):
        bsk_logger.bskError("preformatted %s message")


def test_warning_level_output_is_flushed(capfd):
    """A ``BSK_WARNING`` must reach stdout at log time (issue #1444).

    ``bskLog`` flushes warning-level (and higher) output, so it is observable to
    ``capfd`` before the process exits. Without that flush the C runtime fully
    buffers the line when stdout is not a TTY (pytest capture, pipes, redirection)
    and it would be lost on a crash and invisible to this assertion. The log level
    is set on the instance so the test does not depend on the global default level
    left behind by other tests."""
    bsk_logger = bskLogging.BSKLogger()
    bsk_logger.setLogLevel(bskLogging.BSK_WARNING)

    bsk_logger.bskLog(bskLogging.BSK_WARNING, "flush regression marker")

    out, _ = capfd.readouterr()
    assert "flush regression marker" in out
