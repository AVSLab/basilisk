#
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

"""OMM header datatype definitions."""

import datetime as dt
import re
from dataclasses import dataclass
from typing import Optional


@dataclass
class Header:
    """
    OMM header fields (CCSDS 502.0-B-3 Table 4-1).

    Required fields:
    - ``CCSDS_OMM_VERS``
    - ``CREATION_DATE``
    - ``ORIGINATOR``

    Optional fields:
    - ``COMMENT``
    - ``CLASSIFICATION``
    - ``MESSAGE_ID``
    """

    ccsds_omm_vers: str
    creation_date: dt.datetime
    originator: str
    comment: Optional[str] = None
    classification: Optional[str] = None
    message_id: Optional[str] = None

    def __post_init__(self):
        """Validate required OMM header fields and normalize optional string inputs."""
        # Check CCSDS_OMM_VERS format (e.g. "3.0")
        if not re.fullmatch(r"\d+\.\d+", self.ccsds_omm_vers.strip()):
            raise ValueError(
                "Header.ccsds_omm_vers must be in 'x.y' format, e.g. '3.0'."
            )

        # Check that originator is non-empty
        if not self.originator or not self.originator.strip():
            raise ValueError("Header.originator is required and cannot be empty.")

        # Check that creation_date is timezone-aware and in UTC
        if self.creation_date.tzinfo is None or self.creation_date.utcoffset() is None:
            raise ValueError("Header.creation_date must be timezone-aware and in UTC.")
        if self.creation_date.utcoffset() != dt.timedelta(0):
            raise ValueError("Header.creation_date must be in UTC (offset +00:00).")

        self.ccsds_omm_vers = self.ccsds_omm_vers.strip()
        self.originator = self.originator.strip()
        self.comment = (
            self.comment.strip() if isinstance(self.comment, str) else self.comment
        )
        self.classification = (
            self.classification.strip()
            if isinstance(self.classification, str)
            else self.classification
        )
        self.message_id = (
            self.message_id.strip()
            if isinstance(self.message_id, str)
            else self.message_id
        )

    def to_dict(self) -> dict:
        """
        Export the OMM header using CCSDS OMM keyword names.

        :return: Dictionary with OMM header keyword-value pairs
        """
        return {
            "CCSDS_OMM_VERS": self.ccsds_omm_vers,
            "CREATION_DATE": self.creation_date.isoformat().replace("+00:00", "Z"),
            "ORIGINATOR": self.originator,
            "COMMENT": self.comment,
            "CLASSIFICATION": self.classification,
            "MESSAGE_ID": self.message_id,
        }
