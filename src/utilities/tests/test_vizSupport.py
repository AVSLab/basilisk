from types import SimpleNamespace

import pytest

from Basilisk.utilities import vizSupport


def test_register_vizard_spacecraft_name_rejects_duplicates():
    """Check repeated Vizard spacecraft names are rejected."""
    usedSpacecraftNames = set()

    vizSupport._registerVizardSpacecraftName("hub", usedSpacecraftNames)
    vizSupport._registerVizardSpacecraftName("panel_1", usedSpacecraftNames)

    with pytest.raises(ValueError, match="'hub' is used more than once"):
        vizSupport._registerVizardSpacecraftName("hub", usedSpacecraftNames)


def test_handle_mjscene_registers_each_world_parented_body(monkeypatch):
    """Check one MJScene can emit multiple Vizard spacecraft roots."""

    class FakeSubscriber:
        def subscribeTo(self, message):
            self.message = message

    class FakeVizSpacecraftData:
        def __init__(self):
            self.spacecraftName = ""
            self.parentSpacecraftName = ""
            self.scStateInMsg = FakeSubscriber()

    class FakeVizScData(list):
        def push_back(self, item):
            self.append(item)

    class FakeBody:
        def __init__(self, name):
            self.origin = SimpleNamespace(stateOutMsg=f"{name}_state")

        def getOrigin(self):
            return self.origin

    class FakeMJScene:
        bodyNames = ["hub_1", "hub_2", "panel_2"]
        parents = {"hub_1": "world", "hub_2": "world", "panel_2": "hub_2"}

        def __init__(self):
            self.bodies = {name: FakeBody(name) for name in self.bodyNames}

        def getBodyNames(self):
            return self.bodyNames

        def getBodyParentName(self, name):
            return self.parents[name]

        def getBody(self, name):
            return self.bodies[name]

        def getGeomInfos(self):
            return []

    monkeypatch.setattr(
        vizSupport,
        "vizInterface",
        SimpleNamespace(VizSpacecraftData=FakeVizSpacecraftData),
        raising=False,
    )

    viz = SimpleNamespace(scData=FakeVizScData())
    vizSupport._handleMJScene(
        viz,
        FakeMJScene(),
        SimpleNamespace(),
        0,
        [],
        [],
        [],
        cssList=None,
        genericSensorList=None,
        ellipsoidList=None,
        lightList=None,
        genericStorageList=None,
        transceiverList=None,
        spriteList=None,
        modelDictionaryKeyList=None,
        logoTextureList=None,
        oscOrbitColorList=None,
        trueOrbitColorList=None,
        trueOrbitColorInMsgList=None,
        groundTrackColorList=None,
        groundTrackBodyNameList=None,
        msmInfoList=None,
        usedSpacecraftNames=set(),
    )

    assert [(sc.spacecraftName, sc.parentSpacecraftName) for sc in viz.scData] == [
        ("hub_1", ""),
        ("hub_2", ""),
        ("panel_2", "hub_2"),
    ]


def test_create_custom_models_from_mjscene_scales_cylinders_on_z(monkeypatch):
    """Check MuJoCo cylinders and capsules keep their long axis in Vizard."""

    unused_size = 0.0  # [m]
    capsule_radius = 0.1  # [m]
    capsule_half_length = 0.5  # [m]
    cylinder_radius = 0.2  # [m]
    cylinder_half_length = 1.5  # [m]
    origin = [0.0, 0.0, 0.0]  # [m]
    identity_quat = [1.0, 0.0, 0.0, 0.0]  # [-]
    red_rgba = [1.0, 0.0, 0.0, 1.0]  # [-]

    class FakeMJScene:
        def getGeomInfos(self):
            return [
                SimpleNamespace(
                    bodyName="arm",
                    type=3,
                    size=[capsule_radius, capsule_half_length, unused_size],
                    pos=origin,
                    quat=identity_quat,
                    rgba=red_rgba,
                ),
                SimpleNamespace(
                    bodyName="boom",
                    type=5,
                    size=[cylinder_radius, cylinder_half_length, unused_size],
                    pos=origin,
                    quat=identity_quat,
                    rgba=red_rgba,
                ),
            ]

    custom_models = []

    def capture_custom_model(viz, **kwargs):
        custom_models.append(kwargs)

    monkeypatch.setattr(vizSupport, "createCustomModel", capture_custom_model)

    vizSupport._createCustomModelsFromMJScene(SimpleNamespace(), FakeMJScene())

    assert custom_models[0]["modelPath"] == "CYLINDER"
    assert custom_models[0]["scale"] == pytest.approx([
        2.0 * capsule_radius,
        2.0 * capsule_radius,
        capsule_half_length,
    ])
    assert custom_models[1]["modelPath"] == "CYLINDER"
    assert custom_models[1]["scale"] == pytest.approx([
        2.0 * cylinder_radius,
        2.0 * cylinder_radius,
        cylinder_half_length,
    ])
