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
#   Unit Test Script
#   Module Name: messaging Recorder (large-history storage)
#   Author: robotrocketscience (https://github.com/robotrocketscience)
#   Creation Date: June 30, 2026
#

import numpy as np

from Basilisk.architecture import messaging


def test_recorder_large_history():
    """Record a long history that crosses the storage container's growth boundaries and
    verify every sample round-trips intact.

    The Recorder stores its history in a std::deque (see issue #788). This test guards that
    appending well past the points where the previous std::vector storage would reallocate
    (2**16 = 65536 and beyond) preserves both the recorded times and the recorded payload
    fields exactly. It is a pure correctness-at-scale check with no timing assertions, so it
    is deterministic and not flaky. The recorder is driven directly (rather than through a
    full simulation) purely so the large sample count stays fast.
    """

    sampleCount = 70000  # number of recorded samples; > 2**16 to cross the old reallocation boundary
    timeStep = 1000000  # [ns] simulated time between successive recordings

    payload = messaging.CModuleTemplateMsgPayload()
    msg = messaging.CModuleTemplateMsg()
    recorder = msg.recorder()  # default minimum recording interval of 0 -> records on every update
    recorder.Reset(0)

    expectedTimes = np.empty(sampleCount, dtype=np.uint64)
    expectedData = np.empty((sampleCount, 3), dtype=float)

    for i in range(sampleCount):
        # Stamp each sample with a unique, exactly-representable value so any dropped,
        # duplicated, or corrupted entry across a growth boundary is detected.
        sampleVector = [float(i), float(2 * i), float(3 * i)]
        payload.dataVector = sampleVector
        currentTime = i * timeStep  # [ns]
        msg.write(payload, currentTime)
        recorder.UpdateState(currentTime)

        expectedTimes[i] = currentTime
        expectedData[i] = sampleVector

    # Every sample must be present, in order, and unmodified.
    assert len(recorder.times()) == sampleCount
    np.testing.assert_array_equal(recorder.times(), expectedTimes)
    np.testing.assert_array_equal(recorder.dataVector, expectedData)


if __name__ == "__main__":
    test_recorder_large_history()
    print("test_recorder_large_history passed")
