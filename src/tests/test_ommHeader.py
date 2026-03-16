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

from Basilisk.utilities.orbitDataMessages.omm import Header


def test_omm_header_required_fields():
    header = Header(
        ccsds_omm_vers="3.0",
        creation_date=datetime(2026, 3, 10, 12, 0, 0, tzinfo=timezone.utc),
        originator="JPL",
    )
    header_dict = header.to_dict()
    assert header_dict["CCSDS_OMM_VERS"] == "3.0"
    assert header_dict["CREATION_DATE"].endswith("Z")
    assert header_dict["ORIGINATOR"] == "JPL"


def test_omm_header_optional_fields_exported():
    header = Header(
        ccsds_omm_vers="3.0",
        creation_date=datetime(2026, 3, 10, 12, 0, 0, tzinfo=timezone.utc),
        originator="GSFC",
        comment="This is a comment",
        classification="SBU",
        message_id="OMM 201113719185",
    )
    header_dict = header.to_dict()
    assert header_dict["COMMENT"] == "This is a comment"
    assert header_dict["CLASSIFICATION"] == "SBU"
    assert header_dict["MESSAGE_ID"] == "OMM 201113719185"


@pytest.mark.parametrize("bad_vers", ["3", "3.", "v3.0", "three.zero"])
def test_omm_header_bad_version_raises(bad_vers):
    with pytest.raises(ValueError, match="x.y"):
        Header(
            ccsds_omm_vers=bad_vers,
            creation_date=datetime(2026, 3, 10, 12, 0, 0, tzinfo=timezone.utc),
            originator="JPL",
        )


def test_omm_header_non_utc_raises():
    with pytest.raises(ValueError, match="UTC"):
        Header(
            ccsds_omm_vers="3.0",
            creation_date=datetime(2026, 3, 10, 12, 0, 0),
            originator="JPL",
        )
