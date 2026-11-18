# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

from pathlib import Path

import pytest

from Basilisk.architecture import messaging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import magneticFieldWMM
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities.supportDataTools.dataFetcher import DataFile, get_path


@pytest.fixture
def coefficient_lines():
    """Return the shipped WMM coefficient file as individual lines."""
    path = Path(get_path(DataFile.MagneticFieldData.WMM))
    return path.read_text().splitlines()


def make_module(path):
    """Configure a WMM module with a spacecraft above the equator."""
    module = magneticFieldWMM.MagneticFieldWMM()
    module.configureWMMFile(str(path))
    module.epochDateFractionalYear = 2026.0  # [year]
    state = messaging.SCStatesMsgPayload()
    altitude = 400.0  # [km]
    state.r_BN_N = [
        (orbitalMotion.REQ_EARTH + altitude) * 1000.0, 0.0, 0.0
    ]  # [m]
    module.addSpacecraftToModel(messaging.SCStatesMsg().write(state))
    return module


# Record columns: degree, order, g/h [nT], and dg/dh [nT/year].
@pytest.mark.parametrize("record", [
    "x 1 1.0 2.0 3.0 4.0",
    "1 x 1.0 2.0 3.0 4.0",
    "1 1 x 2.0 3.0 4.0",
    "1 1 1.0 x 3.0 4.0",
    "1 1 1.0 2.0 x 4.0",
    "1 1 1.0 2.0 3.0 x",
    "1 1 1.0 2.0 3.0",
    "-1 -1 1.0 2.0 3.0 4.0",
    "1 -1 1.0 2.0 3.0 4.0",
    "1 2 1.0 2.0 3.0 4.0",
    "70000 0 1.0 2.0 3.0 4.0",
    "4294967297 1 1.0 2.0 3.0 4.0",
    "-4294967295 1 1.0 2.0 3.0 4.0",
    "1 4294967296 1.0 2.0 3.0 4.0",
    "1 -4294967295 1.0 2.0 3.0 4.0",
    "18446744073709551617 1 1.0 2.0 3.0 4.0",
    "-18446744073709551617 1 1.0 2.0 3.0 4.0",
    "1 18446744073709551617 1.0 2.0 3.0 4.0",
    "1 -18446744073709551617 1.0 2.0 3.0 4.0",
    "",
    "\0",
])
def test_invalid_coefficient_record(tmp_path, coefficient_lines, record):
    """Malformed fields and invalid indices fail during module reset."""
    index = next(i for i, line in enumerate(coefficient_lines)
                 if line.split()[:2] == ["1", "1"])
    coefficient_lines[index] = record
    path = tmp_path / "invalid.COF"
    path.write_text("\n".join(coefficient_lines) + "\n")
    module = make_module(path)

    with pytest.raises(BasiliskError, match="WMM unable to load file"):
        module.Reset(0)

    # Failed loads must release the partial model and allow a valid reset.
    module.configureWMMFile(str(get_path(DataFile.MagneticFieldData.WMM)))
    module.Reset(0)
    module.UpdateState(0)
    assert any(module.envOutMsgs[0].read().magField_N)


@pytest.mark.parametrize("header", ["", "invalid WMM", "2025.0"])
def test_invalid_header(tmp_path, coefficient_lines, header):
    """Missing epoch or model-name fields cannot initialize a WMM model."""
    coefficient_lines[0] = header
    path = tmp_path / "invalid_header.COF"
    path.write_text("\n".join(coefficient_lines) + "\n")
    module = make_module(path)
    with pytest.raises(BasiliskError, match="WMM unable to load file"):
        module.Reset(0)


@pytest.mark.parametrize("header_only", [False, True])
def test_empty_model(tmp_path, coefficient_lines, header_only):
    """An empty file or a header without coefficients is rejected."""
    path = tmp_path / "empty.COF"
    path.write_text(coefficient_lines[0] + "\n" if header_only else "")
    module = make_module(path)
    with pytest.raises(BasiliskError, match="WMM unable to load file"):
        module.Reset(0)


@pytest.mark.parametrize("line_ending", ["\n", "\r\n"])
@pytest.mark.parametrize("terminator", [False, True])
def test_valid_file_compatibility(
    tmp_path, coefficient_lines, line_ending, terminator
):
    """Valid line endings and EOF termination preserve the computed field."""
    if not terminator:
        coefficient_lines = [line for line in coefficient_lines
                             if not line.startswith("9999")]
    path = tmp_path / "valid.COF"
    content = line_ending.join(coefficient_lines) + line_ending
    path.write_bytes(content.encode())
    reference = make_module(get_path(DataFile.MagneticFieldData.WMM))
    module = make_module(path)
    for model in (reference, module):
        model.Reset(0)
        model.UpdateState(0)
    expected = reference.envOutMsgs[0].read().magField_N
    assert module.envOutMsgs[0].read().magField_N == expected

    # Failed reinitialization must safely clean up the previous model.
    path.write_text("invalid WMM\n1 0 1.0 0.0 0.0 0.0\n")
    with pytest.raises(BasiliskError, match="WMM unable to load file"):
        module.Reset(0)
    module.configureWMMFile(str(get_path(DataFile.MagneticFieldData.WMM)))
    module.Reset(0)
    module.UpdateState(0)
    assert module.envOutMsgs[0].read().magField_N == expected
