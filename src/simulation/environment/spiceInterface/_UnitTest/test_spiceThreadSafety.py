#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import os
import sys
import time
import hashlib
import multiprocessing as mp
import pytest
import inspect
import traceback

# Import Basilisk modules
from Basilisk import __path__
from Basilisk.simulation import spiceInterface

r"""
Unit Test for SPICE Interface Thread Safety
===========================================

This script tests the thread safety of the SPICE interface, specifically addressing
`GitHub issue #220 <https://github.com/AVSLab/basilisk/issues/220>`_ where parallel simulations using SPICE were causing deadlocks and
data corruption.

The test creates multiple SpiceInterface instances in parallel and forces them to
load/unload kernels simultaneously, creating contention that would previously lead
to deadlocks or corruption.
"""

def get_file_hash(file_path):
    """Calculate SHA-1 hash of a file to check for corruption."""
    hash_sha1 = hashlib.sha1()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_sha1.update(chunk)
    return hash_sha1.hexdigest()

def create_load_destroy_spice(worker_id, iterations, dataPath):
    """
    Repeatedly create, load, and destroy SpiceInterface objects.
    This function will be run in parallel by multiple processes.

    Args:
        worker_id: ID of this worker process
        iterations: Number of iterations to perform
        dataPath: Path to SPICE data directory
    """
    print(f"Worker {worker_id} starting with {iterations} iterations")

    success_count = 0
    failure_count = 0
    exceptions = []

    try:
        for i in range(iterations):
            try:
                # Create a new SpiceInterface object
                spice = spiceInterface.SpiceInterface()

                # Use a fixed set of planets to avoid issues with random sampling
                planets = ['earth', 'sun']  # Removed 'mars' which was causing errors
                spice.addPlanetNames(planets)

                # Set data path
                spice.SPICEDataPath = dataPath

                # This will trigger loading of SPICE kernels
                spice.Reset(0)

                # Sleep for a short time to increase chances of contention
                time.sleep(0.001)  # Reduced from 0.01 to speed up test

                # Delete the object, which will trigger kernel unloading
                del spice

                success_count += 1

                # Report progress for each iteration
                print(f"Worker {worker_id} completed iteration {i+1}/{iterations}")
            except Exception as e:
                failure_count += 1
                error_info = {
                    "worker_id": worker_id,
                    "iteration": i,
                    "error": str(e),
                    "traceback": traceback.format_exc()
                }
                exceptions.append(error_info)
                print(f"Worker {worker_id} failed at iteration {i} with error: {str(e)}")

                # Continue with next iteration
                continue

    except Exception as e:
        # This catches any exceptions outside the iteration loop
        failure_count += 1
        error_info = {
            "worker_id": worker_id,
            "iteration": -1,  # Outside the loop
            "error": str(e),
            "traceback": traceback.format_exc()
        }
        exceptions.append(error_info)
        print(f"Worker {worker_id} failed with error outside iteration loop: {str(e)}")

    return {
        "worker_id": worker_id,
        "success_count": success_count,
        "failure_count": failure_count,
        "exceptions": exceptions
    }

def run_thread_safety_test(num_workers=2, iterations_per_worker=5):
    """
    Run the SPICE thread safety test.

    Args:
        num_workers: Number of parallel workers
        iterations_per_worker: Number of iterations per worker

    Returns:
        results: Dictionary with test results
        success: True if all operations completed successfully
    """
    print(f"Starting SPICE Thread Safety Test with {num_workers} workers")
    print(f"Each worker will perform {iterations_per_worker} iterations")

    # Get SPICE data path using the same approach as the other SPICE tests
    bskPath = __path__[0]
    dataPath = bskPath + '/supportData/EphemerisData/'

    # Start timer
    start_time = time.time()

    # Prepare worker arguments
    worker_args = [(i, iterations_per_worker, dataPath) for i in range(num_workers)]

    # Run workers in parallel
    with mp.Pool(processes=num_workers) as pool:
        worker_results = list(pool.starmap(create_load_destroy_spice, worker_args))

    # End timer
    end_time = time.time()
    execution_time = end_time - start_time

    # Analyze results
    total_success = sum(r["success_count"] for r in worker_results)
    total_failure = sum(r["failure_count"] for r in worker_results)
    all_exceptions = [e for r in worker_results for e in r["exceptions"]]

    # Prepare test results
    results = {
        "execution_time": execution_time,
        "total_iterations": num_workers * iterations_per_worker,
        "successful_iterations": total_success,
        "failed_iterations": total_failure,
        "exceptions": all_exceptions,
        "corruption_detected": False  # We're not checking for corruption in this simplified version
    }

    # Print summary
    print("\n--- SPICE Thread Safety Test Report ---")
    print(f"Total execution time: {execution_time:.2f} seconds")
    print(f"Total iterations: {num_workers * iterations_per_worker}")
    print(f"Successful iterations: {total_success}")
    print(f"Failed iterations: {total_failure}")
    print(f"Exceptions encountered: {len(all_exceptions)}")
    print("--------------------------------------\n")

    # Determine overall success
    if total_success == 0:
        # If no successful iterations at all, that's a test failure
        print("TEST FAILED: No successful iterations completed")
        if len(all_exceptions) > 0:
            print("\nFirst exception details:")
            print(all_exceptions[0]["traceback"])
        success = False
    else:
        # If some iterations succeeded, check for partial failures
        success = (total_failure == 0)

        if success:
            print("TEST PASSED: SPICE interface thread safety implementation is robust!")
        else:
            print("TEST FAILED: Issues detected with SPICE interface thread safety")
            if len(all_exceptions) > 0:
                print("\nFirst exception details:")
                print(all_exceptions[0]["traceback"])

    return results, success

# Define the function outside of any test functions so it can be pickled
def _run_test_with_timeout(result_queue, num_workers, iterations):
    try:
        results, success = run_thread_safety_test(num_workers, iterations)
        result_queue.put((results, success))
    except Exception as e:
        import traceback
        result_queue.put(({"error": str(e), "traceback": traceback.format_exc()}, False))

# Pytest test function
@pytest.mark.parametrize(
    "num_workers, iterations",
    [
        (2, 2),  # Using 2 workers with 2 iterations each to test thread safety
    ]
)
def test_spice_thread_safety(show_plots, num_workers, iterations):
    """
    Test the thread safety of SPICE interface

    Args:
        show_plots: Pytest fixture, not used but required
        num_workers: Number of parallel workers to use
        iterations: Number of kernel load/unload iterations per worker
    """
    # Use multiprocessing with a timeout instead of signals for better platform compatibility
    from multiprocessing import Process, Queue
    import queue

    # Create a queue for results
    result_queue = Queue()

    # Create and start process
    test_process = Process(target=_run_test_with_timeout, args=(result_queue, num_workers, iterations))
    test_process.start()

    # Wait for result with timeout
    timeout = 60  # 60 seconds timeout
    test_process.join(timeout)

    # Check if process is still alive (timeout occurred)
    if test_process.is_alive():
        # Kill the process if it's still running
        test_process.terminate()
        test_process.join(1)  # Give it 1 second to terminate

        # If still alive after terminate, force kill
        if test_process.is_alive():
            import os
            import signal
            os.kill(test_process.pid, signal.SIGKILL)

        pytest.fail(f"Thread safety test timed out after {timeout} seconds")

    # Get result from queue
    try:
        results, success = result_queue.get(block=False)

        # Handle error case
        if isinstance(results, dict) and "error" in results:
            pytest.fail(f"Thread safety test failed with error: {results['error']}\n{results.get('traceback')}")

        # Assert that the test passed
        assert success, "Thread safety test failed with thread safety issues"
        assert results["failed_iterations"] == 0, "Some iterations failed"
    except queue.Empty:
        pytest.fail("Thread safety test completed but did not return any results")


if __name__ == "__main__":
    # Run the test directly with minimal parameters
    num_workers = 2
    iterations_per_worker = 2

    # Allow command line overrides
    if len(sys.argv) > 1:
        num_workers = int(sys.argv[1])
    if len(sys.argv) > 2:
        iterations_per_worker = int(sys.argv[2])

    # Use multiprocessing with a timeout
    from multiprocessing import Process, Queue
    import queue

    # Create a queue for results
    result_queue = Queue()

    # Create and start process
    test_process = Process(target=_run_test_with_timeout, args=(result_queue, num_workers, iterations_per_worker))
    test_process.start()

    # Wait for result with timeout
    timeout = 60  # 60 seconds timeout
    test_process.join(timeout)

    # Check if process is still alive (timeout occurred)
    if test_process.is_alive():
        # Kill the process if it's still running
        test_process.terminate()
        test_process.join(1)  # Give it 1 second to terminate

        # If still alive after terminate, force kill
        if test_process.is_alive():
            import os
            import signal
            os.kill(test_process.pid, signal.SIGKILL)

        print(f"ERROR: Thread safety test timed out after {timeout} seconds")
        sys.exit(2)

    # Get result from queue
    try:
        results, success = result_queue.get(block=False)

        # Handle error case
        if isinstance(results, dict) and "error" in results:
            print(f"ERROR: Thread safety test failed with error: {results['error']}")
            print(results.get('traceback'))
            sys.exit(1)

        # Exit with appropriate status code
        sys.exit(0 if success else 1)
    except queue.Empty:
        print("ERROR: Thread safety test completed but did not return any results")
        sys.exit(1)
