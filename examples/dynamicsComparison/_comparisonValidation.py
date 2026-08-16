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

r"""Validation helpers for dynamics-engine comparison histories."""

import functools
import math

import numpy as np

from Basilisk.utilities import macros


def recorderSampleInterval(taskStep, requestedInterval):
    """Return the recorder interval realizable on a periodic simulation task.

    Args:
        taskStep (float): simulation-task interval [s].
        requestedInterval (float): requested recorder interval [s].

    Returns:
        float: first integer multiple of ``taskStep`` not shorter than the
        requested interval [s].
    """
    taskNanos = macros.sec2nano(float(taskStep))
    requestedNanos = macros.sec2nano(float(requestedInterval))
    if taskNanos <= 0 or requestedNanos <= 0:
        raise ValueError("Recorder task and requested intervals must be positive.")
    multiple = (requestedNanos + taskNanos - 1)//taskNanos
    return multiple*taskNanos*macros.NANO2SEC


def alignedHorizon(requestedHorizon, intervals):
    """Return the largest horizon aligned with every supplied interval.

    Args:
        requestedHorizon (float): desired propagation horizon [s].
        intervals (iterable): task and recorder intervals that must divide the
            returned horizon exactly [s].

    Returns:
        float: aligned propagation horizon [s], no longer than the request.

    Raises:
        ValueError: if the requested horizon or any interval is nonpositive, or
            the request is shorter than one common scheduling quantum.
    """
    horizonNanos = macros.sec2nano(float(requestedHorizon))
    intervalNanos = tuple(
        macros.sec2nano(float(interval)) for interval in intervals
    )
    if horizonNanos <= 0 or not intervalNanos:
        raise ValueError(
            "The requested horizon and at least one interval must be positive."
        )
    if any(interval <= 0 for interval in intervalNanos):
        raise ValueError("Every alignment interval must be positive.")

    commonQuantum = functools.reduce(
        lambda first, second: first*second//math.gcd(first, second),
        intervalNanos,
    )
    alignedNanos = horizonNanos//commonQuantum*commonQuantum
    if alignedNanos == 0:
        raise ValueError(
            f"The requested {requestedHorizon} s horizon is shorter than the "
            f"{commonQuantum*macros.NANO2SEC} s common scheduling quantum."
        )
    return alignedNanos*macros.NANO2SEC


def validateHistory(name, times, expectedFinalTime=None, sampleInterval=0.0,
                    requireFinalSample=False, **histories):
    """Validate one recorded simulation history.

    Args:
        name (str): descriptive history name used in exceptions.
        times (array-like): sample timestamps [s].
        expectedFinalTime (float, optional): requested propagation horizon [s].
        sampleInterval (float, optional): recorder sampling interval [s]. When
            supplied with ``expectedFinalTime``, the complete expected
            nanosecond schedule is enforced.
        requireFinalSample (bool, optional): if True, require the recorder
            schedule to include ``expectedFinalTime`` exactly. Use this when
            the recorder samples every dynamics task step.
        **histories: named arrays whose leading dimension must match ``times``.

    Returns:
        numpy.ndarray: validated one-dimensional timestamp array [s].

    Raises:
        ValueError: if timestamps or history lengths are invalid, or the recorder did not
            reach the requested horizon.
    """
    times = np.asarray(times, dtype=float)
    if times.ndim != 1 or times.size == 0:
        raise ValueError(f"{name} must contain a nonempty one-dimensional timestamp history.")
    if not np.all(np.isfinite(times)):
        raise ValueError(f"{name} contains non-finite timestamps.")
    if np.any(np.diff(times) <= 0.0):
        raise ValueError(f"{name} timestamps must be strictly increasing.")

    for historyName, values in histories.items():
        values = np.asarray(values)
        if values.ndim == 0 or len(values) != len(times):
            raise ValueError(
                f"{name} {historyName} has {len(values) if values.ndim else 0} samples; "
                f"expected {len(times)}."
            )
        if not np.all(np.isfinite(values)):
            raise ValueError(f"{name} {historyName} contains non-finite values.")

    if expectedFinalTime is not None:
        finalNanos = macros.sec2nano(float(expectedFinalTime))
        intervalNanos = macros.sec2nano(float(sampleInterval))
        actualNanos = np.rint(times/macros.NANO2SEC).astype(np.int64)
        if intervalNanos > 0:
            if requireFinalSample and finalNanos % intervalNanos != 0:
                raise ValueError(
                    f"{name} requested final time "
                    f"{finalNanos*macros.NANO2SEC:.16g} s is not divisible by "
                    f"the {intervalNanos*macros.NANO2SEC:.16g} s sample interval."
                )
            expectedCount = finalNanos//intervalNanos + 1
            expectedNanos = np.arange(
                expectedCount, dtype=np.int64)*intervalNanos
            if (
                len(actualNanos) != expectedCount
                or not np.array_equal(actualNanos, expectedNanos)
            ):
                expectedFinalNanos = expectedNanos[-1]
                raise ValueError(
                    f"{name} ended at {times[-1]:.16g} s with "
                    f"{len(actualNanos)} samples; expected "
                    f"{expectedFinalNanos*macros.NANO2SEC:.16g} s and "
                    f"{expectedCount} samples on a "
                    f"{intervalNanos*macros.NANO2SEC:.16g} s schedule."
                )
        elif actualNanos[-1] != finalNanos:
            raise ValueError(
                f"{name} ended at {times[-1]:.16g} s, not the requested "
                f"{finalNanos*macros.NANO2SEC:.16g} s."
            )
    return times


def validateMatchingHistories(name, firstTimes, secondTimes, expectedFinalTime=None,
                              sampleInterval=0.0, requireFinalSample=False):
    """Require two simulations to provide identical, complete timestamp histories.

    Args:
        name (str): descriptive comparison name used in exceptions.
        firstTimes (array-like): first timestamp history [s].
        secondTimes (array-like): second timestamp history [s].
        expectedFinalTime (float, optional): requested propagation horizon [s].
        sampleInterval (float, optional): recorder sampling interval [s].
        requireFinalSample (bool, optional): if True, require both histories to
            include ``expectedFinalTime`` exactly.

    Returns:
        numpy.ndarray: the common timestamp history [s].

    Raises:
        ValueError: if either history is incomplete or their timestamps differ.
    """
    first = validateHistory(
        name + " first", firstTimes, expectedFinalTime, sampleInterval,
        requireFinalSample)
    second = validateHistory(
        name + " second", secondTimes, expectedFinalTime, sampleInterval,
        requireFinalSample)
    if first.shape != second.shape or not np.array_equal(first, second):
        raise ValueError(
            f"{name} timestamp histories differ: {len(first)} versus {len(second)} samples."
        )
    return first


def requireEqualLength(name, *histories):
    """Require all supplied histories to have the same leading dimension.

    Args:
        name (str): descriptive comparison name used in exceptions.
        *histories: arrays or sequences to compare.

    Returns:
        int: common sample count.

    Raises:
        ValueError: if no histories are supplied or their lengths differ.
    """
    if not histories:
        raise ValueError(f"{name} requires at least one history.")
    lengths = tuple(len(history) for history in histories)
    if len(set(lengths)) != 1:
        raise ValueError(f"{name} history lengths differ: {lengths}.")
    return lengths[0]


@functools.lru_cache(maxsize=8)
def _horizonDivisors(horizonNanos):
    """Return every integer-nanosecond step that divides a horizon exactly."""
    divisors = []
    candidate = 1
    while candidate*candidate <= horizonNanos:
        if horizonNanos % candidate == 0:
            divisors.append(candidate)
            if candidate != horizonNanos//candidate:
                divisors.append(horizonNanos//candidate)
        candidate += 1
    return tuple(sorted(divisors))


def snapStepToHorizon(dt, horizon):
    """Return the task step nearest ``dt`` that divides ``horizon`` exactly.

    Args:
        dt (float): requested task step [s].
        horizon (float): comparison horizon [s].

    Returns:
        float: nearest realizable divisor of the horizon [s].

    Raises:
        ValueError: if either argument is nonpositive or the step exceeds the
            horizon.
    """
    stepNanos = macros.sec2nano(float(dt))
    horizonNanos = macros.sec2nano(float(horizon))
    if stepNanos <= 0 or horizonNanos <= 0:
        raise ValueError(
            f"step {dt} s and horizon {horizon} s must both be positive."
        )
    if stepNanos > horizonNanos:
        raise ValueError(
            f"step {dt} s exceeds the {horizon} s comparison horizon."
        )
    if horizonNanos % stepNanos == 0:
        return float(dt)
    nearest = min(
        _horizonDivisors(horizonNanos),
        key=lambda candidate: (abs(candidate-stepNanos), candidate),
    )
    return nearest*macros.NANO2SEC
