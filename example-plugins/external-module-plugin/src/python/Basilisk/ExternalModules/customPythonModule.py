"""
Pure Python example module that mimics the basic Basilisk SysModel contract.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass
class CustomPythonModule:
    """
    Simple stateful example of a Basilisk-like module implemented in Python.
    """

    dummy: float = 0.0
    input_vector: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    data_vector: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    reset_called: bool = False
    update_called: bool = False

    def Reset(self, current_sim_nanos: float) -> None:  # noqa: N802 - Basilisk naming convention
        del current_sim_nanos
        self.reset_called = True
        self.dummy = 0.0
        self.data_vector = [0.0, 0.0, 0.0]

    def UpdateState(self, current_sim_nanos: float, call_time: float) -> None:  # noqa: N802
        del current_sim_nanos
        self.update_called = True
        self.dummy += 1.0
        self.data_vector = [
            self.dummy + self.input_vector[0],
            self.input_vector[1],
            call_time,
        ]


def customPythonModule() -> CustomPythonModule:
    """Backwards compatible constructor for the module."""

    return CustomPythonModule()


__all__ = ["CustomPythonModule", "customPythonModule"]
