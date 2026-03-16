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

"""OMM metadata datatype definitions."""

import datetime as dt
import re
from dataclasses import dataclass
from typing import Optional


@dataclass
class Metadata:
    """
    OMM metadata fields (CCSDS 502.0-B-3 Table 4-2).

    Required fields:
    - ``OBJECT_NAME``
    - ``OBJECT_ID``
    - ``CENTER_NAME``
    - ``REF_FRAME``
    - ``TIME_SYSTEM``
    - ``MEAN_ELEMENT_THEORY``

    Optional fields:
    - ``COMMENT`` (allowed at metadata beginning)
    - ``REF_FRAME_EPOCH`` (conditional)
    """

    object_name: str
    object_id: str
    center_name: str
    ref_frame: str
    time_system: str
    mean_element_theory: str
    comment: Optional[str] = None
    ref_frame_epoch: Optional[dt.datetime] = None

    def __post_init__(self):
        """Validate required OMM metadata fields and normalize string inputs."""

        # Check required string fields are non-empty and of type str
        required_str_fields = {
            "object_name": self.object_name,
            "object_id": self.object_id,
            "center_name": self.center_name,
            "ref_frame": self.ref_frame,
            "time_system": self.time_system,
            "mean_element_theory": self.mean_element_theory,
        }
        for field_name, field_value in required_str_fields.items():
            if not isinstance(field_value, str) or not field_value.strip():
                raise ValueError(
                    f"Metadata.{field_name} is required and cannot be empty."
                )

        object_id_clean = self.object_id.strip()

        # Check OBJECT_ID format (e.g. "2005-046A" or "UNKNOWN")
        if object_id_clean != "UNKNOWN":
            if not re.fullmatch(r"\d{4}-\d{3}[A-Z]+", object_id_clean):
                raise ValueError(
                    "Metadata.object_id must be 'UNKNOWN' or match 'YYYY-NNNP{PP}', "
                    "for example '2005-046A'."
                )

        if self.ref_frame_epoch is not None:
            # Check that ref_frame_epoch is timezone-aware and in UTC if provided
            if (
                self.ref_frame_epoch.tzinfo is None
                or self.ref_frame_epoch.utcoffset() is None
            ):
                raise ValueError(
                    "Metadata.ref_frame_epoch must be timezone-aware and in UTC when provided."
                )
            if self.ref_frame_epoch.utcoffset() != dt.timedelta(0):
                raise ValueError(
                    "Metadata.ref_frame_epoch must be in UTC (offset +00:00)."
                )

        self.object_name = self.object_name.strip()
        self.object_id = object_id_clean
        self.center_name = self.center_name.strip()
        self.ref_frame = self.ref_frame.strip()
        self.time_system = self.time_system.strip()
        self.mean_element_theory = self.mean_element_theory.strip()
        self.comment = (
            self.comment.strip() if isinstance(self.comment, str) else self.comment
        )

    def to_dict(self) -> dict:
        """
        Export OMM metadata using CCSDS OMM keyword names.

        :return: Dictionary with OMM metadata keyword-value pairs
        """
        return {
            "OBJECT_NAME": self.object_name,
            "OBJECT_ID": self.object_id,
            "CENTER_NAME": self.center_name,
            "REF_FRAME": self.ref_frame,
            "TIME_SYSTEM": self.time_system,
            "MEAN_ELEMENT_THEORY": self.mean_element_theory,
            "COMMENT": self.comment,
            "REF_FRAME_EPOCH": (
                self.ref_frame_epoch.isoformat().replace("+00:00", "Z")
                if self.ref_frame_epoch is not None
                else None
            ),
        }
