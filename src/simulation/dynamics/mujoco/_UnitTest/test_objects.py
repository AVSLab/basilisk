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

import os

try:
    from Basilisk.simulation import mujoco

    couldImportMujoco = True
except:
    couldImportMujoco = False

import pytest

TEST_FOLDER = os.path.dirname(__file__)
XML_PATH = f"{TEST_FOLDER}/test_sat.xml"


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_loading():
    """Tests that MJObject are created from the XML as expected"""

    scene = mujoco.MJScene.fromFile(XML_PATH)

    expected_bodies_and_sites = {
        "cube": ("cube_com", "cube_origin"),
        "panel_1": ("panel_1_com", "panel_1_origin", "test_site"),
        "panel_2": ("panel_2_com", "panel_2_origin"),
    }

    for body, sites in expected_bodies_and_sites.items():
        bodyObj = scene.getBody(body)
        for site in sites:
            bodyObj.getSite(site)

    expected_actuators = {
        "panel_1_elevation": scene.getSingleActuator,
        "panel_2_elevation": scene.getSingleActuator,
        "test_act_1": scene.getForceActuator,
        "test_act_2": scene.getForceActuator,
        "test_act_3": scene.getForceTorqueActuator,
        "test_act_4": scene.getForceTorqueActuator,
    }

    for actuator, getter in expected_actuators.items():
        getter(actuator)

    scene.Reset(0)  # this will compile the existing model

    # this will trigger changes in the model
    scene.addTorqueActuator("test_add_1", "panel_2_origin")
    scene.getBody("cube").addSite("test_add_1", [0, 0, 0])

    # Check that we can retrieve the newly added elements
    scene.getTorqueActuator("test_add_1")
    scene.getBody("cube").getSite("test_add_1")

    # Move time forward, which will trigger a re-compile (since
    # we added actuators), and check that the simulation runs ok
    scene.UpdateState(1)


if __name__ == "__main__":
    if True:
        test_loading()
    else:
        pytest.main([__file__])
