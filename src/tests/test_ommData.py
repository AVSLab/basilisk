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
import numpy as np

from Basilisk.utilities.orbitDataMessages.omm import Data
from Basilisk.utilities.orbitDataMessages.omm import CovarianceMatrix
from Basilisk.utilities.orbitDataMessages.omm import MeanKeplerianElements
from Basilisk.utilities.orbitDataMessages.omm import TleRelatedParameters


def _base_mean_elements(**kwargs):
    return MeanKeplerianElements(
        epoch=datetime(2026, 3, 10, 12, 0, 0, tzinfo=timezone.utc),
        eccentricity=0.001,
        inclination=51.6,
        ra_of_asc_node=120.0,
        arg_of_pericenter=85.0,
        mean_anomaly=15.0,
        **kwargs,
    )


def test_omm_data_non_tle_theory_with_semi_major_axis():
    data = Data(
        mean_element_theory="DSST",
        mean_elements=_base_mean_elements(semi_major_axis=7000.0),
    )
    out = data.to_dict()
    assert out["MEAN_KEPLERIAN_ELEMENTS"]["SEMI_MAJOR_AXIS"] == 7000.0


def test_omm_data_non_tle_theory_with_mean_motion():
    data = Data(
        mean_element_theory="DSST",
        mean_elements=_base_mean_elements(mean_motion=15.5),
    )
    out = data.to_dict()
    assert out["MEAN_KEPLERIAN_ELEMENTS"]["MEAN_MOTION"] == 15.5


def test_omm_data_tle_theory_requires_mean_motion_and_tle_params():
    data = Data(
        mean_element_theory="SGP4",
        mean_elements=_base_mean_elements(mean_motion=15.5),
        tle_related_parameters=TleRelatedParameters(
            norad_cat_id=25544,
            bstar_or_bterm=0.0001,
        ),
    )
    out = data.to_dict()
    assert out["MEAN_KEPLERIAN_ELEMENTS"]["MEAN_MOTION"] == 15.5
    assert out["TLE_RELATED_PARAMETERS"]["NORAD_CAT_ID"] == 25544
    assert out["TLE_RELATED_PARAMETERS"]["BSTAR"] == 0.0001


def test_omm_data_rejects_both_axis_and_mean_motion():
    with pytest.raises(ValueError, match="both SEMI_MAJOR_AXIS and MEAN_MOTION"):
        Data(
            mean_element_theory="SGP4",
            mean_elements=_base_mean_elements(semi_major_axis=7000.0, mean_motion=15.5),
            tle_related_parameters=TleRelatedParameters(
                norad_cat_id=25544,
                bstar_or_bterm=0.0001,
            ),
        )


def test_omm_data_requires_mean_motion_dot_for_sgp():
    with pytest.raises(ValueError, match="MEAN_MOTION_DOT"):
        Data(
            mean_element_theory="SGP",
            mean_elements=_base_mean_elements(mean_motion=15.5),
            tle_related_parameters=TleRelatedParameters(
                norad_cat_id=25544,
            ),
        )


def test_omm_data_requires_agom_for_sgp4xp():
    with pytest.raises(ValueError, match="AGOM"):
        Data(
            mean_element_theory="SGP4-XP",
            mean_elements=_base_mean_elements(mean_motion=15.5),
            tle_related_parameters=TleRelatedParameters(
                bstar_or_bterm=0.0001,
            ),
        )


def test_omm_data_sgp4xp_exports_bterm_and_agom():
    data = Data(
        mean_element_theory="SGP4-XP",
        mean_elements=_base_mean_elements(mean_motion=15.5),
        tle_related_parameters=TleRelatedParameters(
            bstar_or_bterm=0.02,
            mean_motion_ddot_or_agom=0.01,
        ),
    )
    out = data.to_dict()
    assert out["TLE_RELATED_PARAMETERS"]["BTERM"] == 0.02
    assert out["TLE_RELATED_PARAMETERS"]["AGOM"] == 0.01
    assert "BSTAR" not in out["TLE_RELATED_PARAMETERS"]
    assert "MEAN_MOTION_DDOT" not in out["TLE_RELATED_PARAMETERS"]


def test_omm_covariance_matrix_numpy_export():
    cov = CovarianceMatrix(matrix=np.eye(6))
    lower = cov.to_lower_triangular_dict()
    assert lower["CX_X"] == 1.0
    assert lower["CY_X"] == 0.0
    assert lower["CZ_DOT_Z_DOT"] == 1.0


def test_omm_covariance_export_none_fields():
    data = Data(
        mean_element_theory="DSST",
        mean_elements=_base_mean_elements(semi_major_axis=7000.0),
        covariance_matrix=CovarianceMatrix(matrix=np.eye(6)),
    )
    out = data.to_dict()
    assert out["COVARIANCE_MATRIX"]["COV_REF_FRAME"] is None
    assert out["COVARIANCE_MATRIX"]["COMMENT"] is None
    assert out["COVARIANCE_MATRIX"]["CX_X"] == 1.0


def test_omm_covariance_matrix_rejects_nonsymmetric():
    bad_matrix = np.eye(6)
    bad_matrix[0, 1] = 5.0
    with pytest.raises(ValueError, match="symmetric"):
        CovarianceMatrix(matrix=bad_matrix)
