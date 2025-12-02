#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle
#  Systems Lab, University of Colorado at Boulder
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

import multiprocessing as mp
import os
import sys
import time
import traceback

import pytest
from Basilisk import __path__
from Basilisk.simulation import spiceInterface

r"""
Unit Test for SPICE Interface Thread Safety
===========================================

This script stress-tests the SPICE interface in parallel, reproducing
the conditions of GitHub issue #220 where parallel simulations using
SPICE could deadlock or corrupt data.

Multiple worker processes repeatedly create and destroy SpiceInterface
instances, forcing concurrent kernel load/unload operations. The test
passes if all workers complete without hangs or unhandled exceptions.
"""

bskPath = __path__[0]


def createLoadDestroySpice(workerId, iterations, dataPath):
    """
    Repeatedly create, reset, and destroy SpiceInterface objects.

    This function is run in parallel by multiple processes. Each worker
    performs ``iterations`` cycles of:

    1. Constructing a SpiceInterface
    2. Configuring planet names and SPICE data path
    3. Calling Reset (which triggers kernel loads)
    4. Brief sleep to increase contention
    5. Deleting the interface (allowing kernels to be released)

    Parameters
    ----------
    workerId : int
        Identifier for this worker process.
    iterations : int
        Number of create/reset/destroy cycles to perform.
    dataPath : str
        Directory containing SPICE kernel data.

    Returns
    -------
    dict
        Summary for this worker with counts of successes, failures, and
        a list of captured exception details.
    """
    print(f"Worker {workerId} starting with {iterations} iterations")

    successCount = 0
    failureCount = 0
    exceptionList = []

    try:
        for iteration in range(iterations):
            try:
                # Create a new SpiceInterface
                spiceObj = spiceInterface.SpiceInterface()

                # Use a fixed planet set to avoid random differences
                planets = ["earth", "sun"]
                spiceObj.addPlanetNames(planets)

                # Configure SPICE data path and trigger kernel loads
                spiceObj.SPICEDataPath = dataPath
                spiceObj.Reset(0)

                # Short sleep to encourage overlap among workers
                time.sleep(0.001)

                # Drop reference so the object can be destroyed
                del spiceObj

                successCount += 1
                print(
                    f"Worker {workerId} completed iteration "
                    f"{iteration + 1}/{iterations}"
                )
            except Exception as exc:
                failureCount += 1
                errorInfo = {
                    "workerId": workerId,
                    "iteration": iteration,
                    "error": str(exc),
                    "traceback": traceback.format_exc(),
                }
                exceptionList.append(errorInfo)
                print(
                    f"Worker {workerId} failed at iteration {iteration} "
                    f"with error: {exc}"
                )
                # Continue with next iteration
                continue

    except Exception as exc:
        # Catch any exception outside the main loop
        failureCount += 1
        errorInfo = {
            "workerId": workerId,
            "iteration": -1,  # Outside the loop
            "error": str(exc),
            "traceback": traceback.format_exc(),
        }
        exceptionList.append(errorInfo)
        print(f"Worker {workerId} failed with error outside iteration loop: {exc}")

    return {
        "workerId": workerId,
        "successCount": successCount,
        "failureCount": failureCount,
        "exceptions": exceptionList,
    }


def runThreadSafetyTest(numWorkers=2, iterationsPerWorker=5):
    """
    Run the SPICE thread-safety stress test.

    Parameters
    ----------
    numWorkers : int
        Number of parallel worker processes to launch.
    iterationsPerWorker : int
        Number of create/reset/destroy cycles per worker.

    Returns
    -------
    results : dict
        Aggregate statistics over all workers.
    success : bool
        True if all iterations completed without failure, False otherwise.
    """
    print(f"Starting SPICE Thread Safety Test with {numWorkers} workers")
    print(f"Each worker will perform {iterationsPerWorker} iterations")

    dataPath = bskPath + "/supportData/EphemerisData/"

    startTime = time.time()

    workerArgs = [
        (workerId, iterationsPerWorker, dataPath) for workerId in range(numWorkers)
    ]

    with mp.Pool(processes=numWorkers) as pool:
        workerResults = list(pool.starmap(createLoadDestroySpice, workerArgs))

    endTime = time.time()
    executionTime = endTime - startTime

    totalSuccess = sum(r["successCount"] for r in workerResults)
    totalFailure = sum(r["failureCount"] for r in workerResults)
    allExceptions = [e for r in workerResults for e in r["exceptions"]]

    results = {
        "executionTime": executionTime,
        "totalIterations": numWorkers * iterationsPerWorker,
        "successfulIterations": totalSuccess,
        "failedIterations": totalFailure,
        "exceptions": allExceptions,
    }

    print("\n--- SPICE Thread Safety Test Report ---")
    print(f"Total execution time: {executionTime:.2f} seconds")
    print(f"Total iterations: {numWorkers * iterationsPerWorker}")
    print(f"Successful iterations: {totalSuccess}")
    print(f"Failed iterations: {totalFailure}")
    print(f"Exceptions encountered: {len(allExceptions)}")
    print("--------------------------------------\n")

    if totalSuccess == 0:
        print("TEST FAILED: No successful iterations completed")
        if len(allExceptions) > 0:
            print("\nFirst exception details:")
            print(allExceptions[0]["traceback"])
        success = False
    else:
        success = totalFailure == 0
        if success:
            print("TEST PASSED: SPICE interface thread safety looks robust")
        else:
            print("TEST FAILED: Issues detected with SPICE interface thread safety")
            if len(allExceptions) > 0:
                print("\nFirst exception details:")
                print(allExceptions[0]["traceback"])

    return results, success


def _runTestWithTimeout(resultQueue, numWorkers, iterationsPerWorker):
    """
    Helper used as a process entry point to run the test with a timeout.

    This is defined at module level so that it is picklable by
    multiprocessing on all supported platforms.
    """
    try:
        results, success = runThreadSafetyTest(numWorkers, iterationsPerWorker)
        resultQueue.put((results, success))
    except Exception as exc:
        resultQueue.put(
            (
                {
                    "error": str(exc),
                    "traceback": traceback.format_exc(),
                },
                False,
            )
        )


@pytest.mark.flaky(reruns=3)
@pytest.mark.parametrize(
    "numWorkers, iterationsPerWorker",
    [
        (10, 3),
    ],
)
def testSpiceThreadSafety(numWorkers, iterationsPerWorker):
    """
    Pytest entry point for the SPICE thread-safety test.

    Parameters
    ----------
    numWorkers : int
        Number of parallel worker processes.
    iterationsPerWorker : int
        Number of load/unload cycles per worker.
    """
    import queue
    from multiprocessing import Process, Queue

    resultQueue = Queue()
    testProcess = Process(
        target=_runTestWithTimeout,
        args=(resultQueue, numWorkers, iterationsPerWorker),
    )
    testProcess.start()

    timeoutSeconds = 60
    testProcess.join(timeoutSeconds)

    if testProcess.is_alive():
        # Hard timeout: kill the worker process and fail the test
        testProcess.terminate()
        testProcess.join(1)
        if testProcess.is_alive():
            os.kill(testProcess.pid, 9)
        pytest.fail(f"Thread safety test timed out after {timeoutSeconds} seconds")

    try:
        results, success = resultQueue.get(block=False)

        if isinstance(results, dict) and "error" in results:
            pytest.fail(
                "Thread safety test failed with error: "
                f"{results['error']}\n{results.get('traceback')}"
            )

        assert success, "Thread safety test reported thread-safety issues"
        assert results["failedIterations"] == 0, (
            "Some iterations failed in the thread-safety test"
        )
    except queue.Empty:
        pytest.fail("Thread safety test completed but did not return any results")


if __name__ == "__main__":
    import queue
    from multiprocessing import Process, Queue

    numWorkers = 50
    iterationsPerWorker = 3

    if len(sys.argv) > 1:
        numWorkers = int(sys.argv[1])
    if len(sys.argv) > 2:
        iterationsPerWorker = int(sys.argv[2])

    resultQueue = Queue()
    testProcess = Process(
        target=_runTestWithTimeout,
        args=(resultQueue, numWorkers, iterationsPerWorker),
    )
    testProcess.start()

    timeoutSeconds = 60
    testProcess.join(timeoutSeconds)

    if testProcess.is_alive():
        testProcess.terminate()
        testProcess.join(1)
        if testProcess.is_alive():
            os.kill(testProcess.pid, 9)
        print(f"ERROR: Thread safety test timed out after {timeoutSeconds} seconds")
        sys.exit(2)

    try:
        results, success = resultQueue.get(block=False)

        if isinstance(results, dict) and "error" in results:
            print(f"ERROR: Thread safety test failed with error: {results['error']}")
            print(results.get("traceback"))
            sys.exit(1)

        sys.exit(0 if success else 1)
    except queue.Empty:
        print("ERROR: Thread safety test completed but did not return results")
        sys.exit(1)
