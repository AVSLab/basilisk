# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# This file is distributed under the ISC License in LICENSE.

"""Run naming integration tests through the public policy flag."""

import pytest


@pytest.fixture(params=[False, True], ids=["legacy", "manager-local"])
def manager_local(request):
    """Exercise both naming policies in ordinary pytest and CTest runs."""
    return request.param
