#
# Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


from unittest.mock import MagicMock

from Basilisk.utilities.SimulationBaseClass import (
    EventHandlerClass,
    methodizeAction,
    methodizeCondition,
)


def test_methodize_condition():
    simBase = MagicMock()
    simBase.condition=True
    simBase.value = 1.0
    condition_fn = methodizeCondition(["self.condition", "self.value==1"])
    assert condition_fn(simBase) is True

    simBase.condition=False
    assert condition_fn(simBase) is False


def test_methodize_action():
    simBase = MagicMock()
    simBase.value = 0.0

    action_fn = methodizeAction(["self.value = 1", "self.action()"])
    action_fn(simBase)

    assert simBase.value == 1.0
    simBase.action.assert_called_once()

def test_event_handler():
    simBase = MagicMock()
    simBase.TotalSim.CurrentNanos = 1e10

    handler = EventHandlerClass(
        eventName="event",
        eventActive=True,
        conditionFunction=lambda self: self.condition,
        actionFunction=lambda self: self.action(),
    )

    # Check not triggered
    simBase.condition=False
    handler.checkEvent(simBase)
    simBase.action.assert_not_called()

    # Check triggered
    simBase.condition=True
    handler.checkEvent(simBase)
    simBase.action.assert_called_once()


if __name__ == "__main__":
    test_methodize_condition()
    test_methodize_action()
    test_event_handler()
