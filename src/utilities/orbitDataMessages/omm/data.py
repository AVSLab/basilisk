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

"""OMM data datatype definitions (CCSDS 502.0-B-3 section 4.2.4)."""

import datetime as dt
from dataclasses import dataclass
from typing import Dict, Optional
import numpy as np


@dataclass
class MeanKeplerianElements:
    """Mean Keplerian elements block."""

    epoch: dt.datetime
    eccentricity: float
    inclination: float
    ra_of_asc_node: float
    arg_of_pericenter: float
    mean_anomaly: float
    semi_major_axis: Optional[float] = None
    mean_motion: Optional[float] = None
    gm: Optional[float] = None
    comment: Optional[str] = None

    def __post_init__(self):
        if self.epoch.tzinfo is None or self.epoch.utcoffset() is None:
            raise ValueError("MeanKeplerianElements.epoch must be timezone-aware UTC.")
        if self.epoch.utcoffset() != dt.timedelta(0):
            raise ValueError(
                "MeanKeplerianElements.epoch must use UTC (offset +00:00)."
            )


@dataclass
class SpacecraftParameters:
    """Spacecraft parameters block."""

    mass: Optional[float] = None
    solar_rad_area: Optional[float] = None
    solar_rad_coeff: Optional[float] = None
    drag_area: Optional[float] = None
    drag_coeff: Optional[float] = None
    comment: Optional[str] = None


@dataclass
class TleRelatedParameters:
    """TLE-related parameters block."""

    ephemeris_type: Optional[int] = None
    classification_type: Optional[str] = None
    norad_cat_id: Optional[int] = None
    element_set_no: Optional[int] = None
    rev_at_epoch: Optional[int] = None
    bstar_or_bterm: Optional[float] = None
    mean_motion_dot: Optional[float] = None
    mean_motion_ddot_or_agom: Optional[float] = None
    comment: Optional[str] = None


@dataclass
class CovarianceMatrix:
    """Position/velocity covariance matrix block as a 6x6 NumPy matrix."""

    matrix: np.ndarray
    cov_ref_frame: Optional[str] = None
    comment: Optional[str] = None

    def __post_init__(self):
        self.matrix = np.asarray(self.matrix, dtype=float)
        if self.matrix.shape != (6, 6):
            raise ValueError("CovarianceMatrix.matrix must have shape (6, 6).")
        if not np.allclose(self.matrix, self.matrix.T, rtol=0.0, atol=1e-14):
            raise ValueError("CovarianceMatrix.matrix must be symmetric.")

    @classmethod
    def from_lower_triangular(
        cls,
        values: list[float],
        cov_ref_frame: Optional[str] = None,
        comment: Optional[str] = None,
    ) -> "CovarianceMatrix":
        """Construct from 21 lower-triangular values (row by row, left to right)."""
        if len(values) != 21:
            raise ValueError(f"Expected 21 lower-triangular values, got {len(values)}.")
        matrix = np.zeros((6, 6), dtype=float)
        matrix[np.tril_indices(6)] = values
        matrix += matrix.T - np.diag(matrix.diagonal())
        return cls(matrix=matrix, cov_ref_frame=cov_ref_frame, comment=comment)

    _LOWER_TRI_KEYS = [
        "CX_X",
        "CY_X",
        "CY_Y",
        "CZ_X",
        "CZ_Y",
        "CZ_Z",
        "CX_DOT_X",
        "CX_DOT_Y",
        "CX_DOT_Z",
        "CX_DOT_X_DOT",
        "CY_DOT_X",
        "CY_DOT_Y",
        "CY_DOT_Z",
        "CY_DOT_X_DOT",
        "CY_DOT_Y_DOT",
        "CZ_DOT_X",
        "CZ_DOT_Y",
        "CZ_DOT_Z",
        "CZ_DOT_X_DOT",
        "CZ_DOT_Y_DOT",
        "CZ_DOT_Z_DOT",
    ]

    def to_lower_triangular_dict(self) -> dict:
        """Export the 6x6 covariance matrix to CCSDS lower-triangular OMM fields."""
        values = self.matrix[np.tril_indices(6)]
        return dict(zip(self._LOWER_TRI_KEYS, values))


@dataclass
class Data:
    """
    Top-level OMM data container with conditional validation rules.

    The validation logic focuses on section 4.2.4 high-level constraints:
    - Mean element axis representation: ``SEMI_MAJOR_AXIS`` or ``MEAN_MOTION``
    - TLE-based theory conditionals for TLE-related parameters
    - Optional covariance block as complete lower-triangular representation
    """

    mean_element_theory: str
    mean_elements: MeanKeplerianElements
    spacecraft_parameters: Optional[SpacecraftParameters] = None
    tle_related_parameters: Optional[TleRelatedParameters] = None
    covariance_matrix: Optional[CovarianceMatrix] = None
    user_defined_parameters: Optional[Dict[str, str]] = None

    def __post_init__(self):
        self.mean_element_theory = self.mean_element_theory.strip().upper()
        (
            self._check_axis_representation()
            ._check_sgp_sgp4()
            ._check_sgp4_bstar()
            ._check_sgp_ppt3_mean_motion_dot()
            ._check_sgp4xp()
        )

    def _check_axis_representation(self) -> "Data":
        has_both = (
            self.mean_elements.mean_motion is not None
            and self.mean_elements.semi_major_axis is not None
        )
        has_neither = (
            self.mean_elements.mean_motion is None
            and self.mean_elements.semi_major_axis is None
        )
        if has_both:
            raise ValueError(
                "Mean elements must not include both SEMI_MAJOR_AXIS and MEAN_MOTION."
            )
        if has_neither:
            raise ValueError(
                "Mean elements must include one of SEMI_MAJOR_AXIS or MEAN_MOTION."
            )
        return self

    def _check_sgp_sgp4(self) -> "Data":
        if self.mean_element_theory not in {"SGP", "SGP4"}:
            return self
        if self.mean_elements.mean_motion is None:
            raise ValueError("SGP/SGP4 mean element theory requires MEAN_MOTION.")
        if self.tle_related_parameters is None:
            raise ValueError(
                "TLE-related parameters are required for SGP/SGP4 theories."
            )
        if self.tle_related_parameters.norad_cat_id is None:
            raise ValueError("NORAD_CAT_ID is required for SGP/SGP4 theories.")
        return self

    def _check_sgp4_bstar(self) -> "Data":
        if self.mean_element_theory != "SGP4":
            return self
        if (
            self.tle_related_parameters is None
            or self.tle_related_parameters.bstar_or_bterm is None
        ):
            raise ValueError("BSTAR is required when MEAN_ELEMENT_THEORY is SGP4.")
        return self

    def _check_sgp_ppt3_mean_motion_dot(self) -> "Data":
        if self.mean_element_theory not in {"SGP", "PPT3"}:
            return self
        if (
            self.tle_related_parameters is None
            or self.tle_related_parameters.mean_motion_dot is None
        ):
            raise ValueError(
                "MEAN_MOTION_DOT is required when MEAN_ELEMENT_THEORY is SGP or PPT3."
            )
        return self

    def _check_sgp4xp(self) -> "Data":
        if self.mean_element_theory != "SGP4-XP":
            return self
        if self.tle_related_parameters is None:
            raise ValueError("TLE-related parameters are required for SGP4-XP theory.")
        if self.tle_related_parameters.bstar_or_bterm is None:
            raise ValueError("BTERM is required when MEAN_ELEMENT_THEORY is SGP4-XP.")
        if self.tle_related_parameters.mean_motion_ddot_or_agom is None:
            raise ValueError("AGOM is required when MEAN_ELEMENT_THEORY is SGP4-XP.")
        return self

    def to_dict(self) -> dict:
        """Export OMM data block(s) as CCSDS keyword dictionaries."""
        me = self.mean_elements
        bstar_key = "BTERM" if self.mean_element_theory == "SGP4-XP" else "BSTAR"
        ddot_key = (
            "AGOM" if self.mean_element_theory == "SGP4-XP" else "MEAN_MOTION_DDOT"
        )

        out = {
            "MEAN_KEPLERIAN_ELEMENTS": {
                "COMMENT": me.comment,
                "EPOCH": me.epoch.isoformat().replace("+00:00", "Z"),
                "SEMI_MAJOR_AXIS": me.semi_major_axis,
                "MEAN_MOTION": me.mean_motion,
                "ECCENTRICITY": me.eccentricity,
                "INCLINATION": me.inclination,
                "RA_OF_ASC_NODE": me.ra_of_asc_node,
                "ARG_OF_PERICENTER": me.arg_of_pericenter,
                "MEAN_ANOMALY": me.mean_anomaly,
                "GM": me.gm,
            },
        }

        if self.spacecraft_parameters is not None:
            sp = self.spacecraft_parameters
            out["SPACECRAFT_PARAMETERS"] = {
                "COMMENT": sp.comment,
                "MASS": sp.mass,
                "SOLAR_RAD_AREA": sp.solar_rad_area,
                "SOLAR_RAD_COEFF": sp.solar_rad_coeff,
                "DRAG_AREA": sp.drag_area,
                "DRAG_COEFF": sp.drag_coeff,
            }

        if self.tle_related_parameters is not None:
            tp = self.tle_related_parameters
            out["TLE_RELATED_PARAMETERS"] = {
                "COMMENT": tp.comment,
                "EPHEMERIS_TYPE": tp.ephemeris_type,
                "CLASSIFICATION_TYPE": tp.classification_type,
                "NORAD_CAT_ID": tp.norad_cat_id,
                "ELEMENT_SET_NO": tp.element_set_no,
                "REV_AT_EPOCH": tp.rev_at_epoch,
                bstar_key: tp.bstar_or_bterm,
                "MEAN_MOTION_DOT": tp.mean_motion_dot,
                ddot_key: tp.mean_motion_ddot_or_agom,
            }

        if self.covariance_matrix is not None:
            covariance_dict = self.covariance_matrix.to_lower_triangular_dict()
            covariance_dict["COV_REF_FRAME"] = self.covariance_matrix.cov_ref_frame
            covariance_dict["COMMENT"] = self.covariance_matrix.comment
            out["COVARIANCE_MATRIX"] = covariance_dict

        if self.user_defined_parameters is not None:
            out["USER_DEFINED_PARAMETERS"] = dict(self.user_defined_parameters)

        return out
