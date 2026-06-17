from copy import copy
from types import SimpleNamespace

import pytest

from Basilisk.utilities import vizSupport


class NonIterableGeomInfos:
    """Mimic SWIG vectors that are safe to index but unsafe to iterate."""

    def __init__(self, geomInfos):
        self.geomInfos = geomInfos

    def __len__(self):
        return len(self.geomInfos)

    def __getitem__(self, index):
        return self.geomInfos[index]

    def __iter__(self):
        raise AssertionError("MJGeomInfo vectors should be indexed, not iterated")


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
            return NonIterableGeomInfos([
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
            ])

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


def test_create_custom_models_from_mjscene_keeps_multi_geom_bodies(monkeypatch):
    """Check additional MuJoCo geoms remain visible on one Vizard body."""

    mj_geom_cylinder = 5  # [-]
    mj_geom_box = 6  # [-]
    panel_half_extents = [1.0, 0.05, 0.8]  # [m]
    bar_radius = 0.075  # [m]
    bar_half_length = 1.0  # [m]
    unused_size = 0.0  # [m]
    panel_position = [0.0, 0.0, 1.0]  # [m]
    bar_position = [0.0, 0.0, 0.0]  # [m]
    identity_quat = [1.0, 0.0, 0.0, 0.0]  # [-]
    green_rgba = [0.0, 1.0, 0.0, 1.0]  # [-]
    cyan_rgba = [0.0, 1.0, 1.0, 1.0]  # [-]

    class FakeSubscriber:
        def subscribeTo(self, message):
            self.message = message

    class FakeMultiShape:
        pass

    class FakeMultiShapeInfo:
        def __init__(self):
            self.msmList = []

    class FakeVizSpacecraftData:
        def __init__(self):
            self.spacecraftName = ""
            self.parentSpacecraftName = ""
            self.scStateInMsg = FakeSubscriber()
            self.msmInfo = FakeMultiShapeInfo()

    class FakeVizScData(list):
        def __iter__(self):
            for item in list.__iter__(self):
                yield copy(item)

        def push_back(self, item):
            self.append(copy(item))

    class FakeBody:
        def __init__(self, name):
            self.origin = SimpleNamespace(stateOutMsg=f"{name}_state")

        def getOrigin(self):
            return self.origin

    class FakeMJScene:
        bodyNames = ["panel"]

        def __init__(self):
            self.bodies = {name: FakeBody(name) for name in self.bodyNames}

        def getBodyNames(self):
            return self.bodyNames

        def getBodyParentName(self, name):
            return "world"

        def getBody(self, name):
            return self.bodies[name]

        def getGeomInfos(self):
            return NonIterableGeomInfos([
                SimpleNamespace(
                    bodyName="panel",
                    type=mj_geom_box,
                    size=panel_half_extents,
                    pos=panel_position,
                    quat=identity_quat,
                    rgba=green_rgba,
                ),
                SimpleNamespace(
                    bodyName="panel",
                    type=mj_geom_cylinder,
                    size=[bar_radius, bar_half_length, unused_size],
                    pos=bar_position,
                    quat=identity_quat,
                    rgba=cyan_rgba,
                ),
            ])

    custom_models = []

    def capture_custom_model(viz, **kwargs):
        custom_models.append(kwargs)

    monkeypatch.setattr(vizSupport, "createCustomModel", capture_custom_model)
    monkeypatch.setattr(vizSupport, "mjSceneMultiShapeList", [])
    monkeypatch.setattr(
        vizSupport,
        "vizInterface",
        SimpleNamespace(
            VizSpacecraftData=FakeVizSpacecraftData,
            MultiShape=FakeMultiShape,
            MultiShapeInfo=FakeMultiShapeInfo,
            IntVector=list,
            MultiShapeVector=list,
        ),
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

    assert len(viz.scData) == 1
    assert len(custom_models) == 1
    assert custom_models[0]["modelPath"] == "CUBE"
    assert custom_models[0]["simBodiesToModify"] == ["panel"]
    assert custom_models[0]["scale"] == pytest.approx([
        2.0 * panel_half_extents[0],
        2.0 * panel_half_extents[1],
        2.0 * panel_half_extents[2],
    ])

    assert len(viz.scData[0].msmInfo.msmList) == 1
    assert len(vizSupport.mjSceneMultiShapeList) == 1
    panel_bar = viz.scData[0].msmInfo.msmList[0]
    assert panel_bar.shape == "CYLINDER"
    assert panel_bar.dimensions == pytest.approx([
        2.0 * bar_radius,
        2.0 * bar_radius,
        bar_half_length,
    ])
    assert panel_bar.position == pytest.approx(bar_position)
    assert panel_bar.rotation == pytest.approx([0.0, 0.0, 0.0])
    assert panel_bar.positiveColor == [0, 255, 255, 255]
