#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import pytest
from datetime import datetime, timezone

from Basilisk.utilities.orbitDataMessages.omm import Metadata


def test_omm_metadata_required_fields():
    metadata = Metadata(
        object_name="TelKom 2",
        object_id="2005-046A",
        center_name="EARTH",
        ref_frame="TEME",
        time_system="UTC",
        mean_element_theory="SGP4",
    )
    metadata_dict = metadata.to_dict()
    assert metadata_dict["OBJECT_NAME"] == "TelKom 2"
    assert metadata_dict["OBJECT_ID"] == "2005-046A"
    assert metadata_dict["CENTER_NAME"] == "EARTH"
    assert metadata_dict["REF_FRAME"] == "TEME"
    assert metadata_dict["TIME_SYSTEM"] == "UTC"
    assert metadata_dict["MEAN_ELEMENT_THEORY"] == "SGP4"


def test_omm_metadata_optional_fields_exported():
    metadata = Metadata(
        object_name="INMARSAT 4-F2",
        object_id="UNKNOWN",
        center_name="EARTH",
        ref_frame="ICRF",
        time_system="UTC",
        mean_element_theory="SGP4",
        comment="This is a comment",
        ref_frame_epoch=datetime(2026, 3, 10, 12, 0, 0, tzinfo=timezone.utc),
    )
    metadata_dict = metadata.to_dict()
    assert metadata_dict["COMMENT"] == "This is a comment"
    assert metadata_dict["REF_FRAME_EPOCH"].endswith("Z")


@pytest.mark.parametrize("bad_object_id", ["2005-46A", "05-046A", "2005-046", "ABC"])
def test_omm_metadata_bad_object_id_raises(bad_object_id):
    with pytest.raises(ValueError, match="YYYY-NNNP"):
        Metadata(
            object_name="Spaceway 2",
            object_id=bad_object_id,
            center_name="EARTH",
            ref_frame="TEME",
            time_system="UTC",
            mean_element_theory="SGP4",
        )


def test_omm_metadata_non_utc_ref_frame_epoch_raises():
    with pytest.raises(ValueError, match="UTC"):
        Metadata(
            object_name="Spaceway 2",
            object_id="2003-022A",
            center_name="EARTH",
            ref_frame="TEME",
            time_system="UTC",
            mean_element_theory="SGP4",
            ref_frame_epoch=datetime(2026, 3, 10, 12, 0, 0),
        )
