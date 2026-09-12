# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# This file is distributed under the ISC License in LICENSE.

"""Share the gated naming bridge across Python integration tests."""

import importlib.util
import os

import pytest


@pytest.fixture
def naming_support():
    """Load the opt-in bridge from the CMake test directory, outside the package."""
    spec = importlib.util.find_spec("effectorNamingTestSupport")
    if spec is None:
        if os.environ.get("BSK_REQUIRE_NAMING_TEST_SUPPORT"):
            pytest.fail("Build the effectorNamingTestSupport target before running this CTest.")
        pytest.skip("Run effectorNamingPython through CTest to test the gated policy.")
    import effectorNamingTestSupport

    return effectorNamingTestSupport


@pytest.fixture(params=[False, True], ids=["legacy", "manager-local"])
def manager_local(request):
    """Keep legacy cases runnable even when the internal test bridge is not built."""
    if request.param:
        request.getfixturevalue("naming_support")
    return request.param
