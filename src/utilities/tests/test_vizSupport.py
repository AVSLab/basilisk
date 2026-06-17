import pytest

from Basilisk.utilities import vizSupport


def test_register_vizard_spacecraft_name_rejects_duplicates():
    """Check repeated Vizard spacecraft names are rejected."""
    usedSpacecraftNames = set()

    vizSupport._registerVizardSpacecraftName("hub", usedSpacecraftNames)
    vizSupport._registerVizardSpacecraftName("panel_1", usedSpacecraftNames)

    with pytest.raises(ValueError, match="'hub' is used more than once"):
        vizSupport._registerVizardSpacecraftName("hub", usedSpacecraftNames)
