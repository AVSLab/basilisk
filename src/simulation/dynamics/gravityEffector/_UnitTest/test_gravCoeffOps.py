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

import csv
import importlib.util
import inspect
from pathlib import Path

import numpy as np
import pytest

from Basilisk.simulation import gravityEffector

GGM03S_PATH = Path(__file__).resolve().with_name("GGM03S.txt")
GRAV_COEFF_OPS_PATH = Path(__file__).resolve().parents[1] / "gravCoeffOps.py"

loadGravFromFileToList = gravityEffector.loadGravFromFileToList
loadPolyFromFileToList = gravityEffector.loadPolyFromFileToList

loadedGravCoeffOpsPath = Path(inspect.getsourcefile(loadGravFromFileToList)).resolve()
if loadedGravCoeffOpsPath != GRAV_COEFF_OPS_PATH:
    gravCoeffOpsSpec = importlib.util.spec_from_file_location(
        "gravCoeffOps", GRAV_COEFF_OPS_PATH
    )
    if gravCoeffOpsSpec is None or gravCoeffOpsSpec.loader is None:
        raise ImportError(f"Unable to load module from {GRAV_COEFF_OPS_PATH}")
    gravCoeffOps = importlib.util.module_from_spec(gravCoeffOpsSpec)
    gravCoeffOpsSpec.loader.exec_module(gravCoeffOps)
    gravityEffector.loadGravFromFileToList = gravCoeffOps.loadGravFromFileToList
    gravityEffector.loadPolyFromFileToList = gravCoeffOps.loadPolyFromFileToList
    loadGravFromFileToList = gravityEffector.loadGravFromFileToList
    loadPolyFromFileToList = gravityEffector.loadPolyFromFileToList


def _loadReferenceCoefficients(filePath: Path, maxDegree: int):
    with filePath.open("r", newline="") as gravFile:
        gravReader = csv.reader(gravFile, delimiter=",")
        firstRow = next(gravReader)
        radEquator = float(firstRow[0])
        mu = float(firstRow[1])
        maxDegreeFile = int(firstRow[3])
        maxOrderFile = int(firstRow[4])

        cRef = [[0.0] * (degree + 1) for degree in range(maxDegree + 1)]
        sRef = [[0.0] * (degree + 1) for degree in range(maxDegree + 1)]

        for gravRow in gravReader:
            degree = int(gravRow[0])
            order = int(gravRow[1])
            if degree > maxDegree:
                break
            cRef[degree][order] = float(gravRow[2])
            sRef[degree][order] = float(gravRow[3])

    return cRef, sRef, mu, radEquator, maxDegreeFile, maxOrderFile


def _loadHeaderAndRows(filePath: Path, maxDegree: int):
    with filePath.open("r", newline="") as gravFile:
        gravReader = csv.reader(gravFile, delimiter=",")
        header = next(gravReader)
        rows = []
        for gravRow in gravReader:
            if int(gravRow[0]) > maxDegree:
                break
            rows.append(gravRow)

    return header, rows


def _writeTempGravityFile(tmpPath: Path, header, rows):
    tempGravityFile = tmpPath / "temporaryGravityFile.csv"
    with tempGravityFile.open("w", newline="") as gravFile:
        gravWriter = csv.writer(gravFile)
        gravWriter.writerow(header)
        gravWriter.writerows(rows)

    return tempGravityFile


def testLoadGravFromFileToListRespectsMaxDegree():
    """Verify spherical-harmonic loading is truncated to the requested degree."""
    maxDegree = 8

    cList, sList, _, _ = loadGravFromFileToList(str(GGM03S_PATH), maxDeg=maxDegree)

    assert len(cList) == maxDegree + 1
    assert len(sList) == maxDegree + 1
    for degree in range(maxDegree + 1):
        assert len(cList[degree]) == degree + 1
        assert len(sList[degree]) == degree + 1


def testLoadGravFromFileToListIncludesRequestedLastDegree():
    """Verify coefficients for the highest requested degree are loaded."""
    maxDegree = 20

    cRef, sRef, muRef, radRef, _, _ = _loadReferenceCoefficients(
        GGM03S_PATH, maxDegree
    )
    cList, sList, mu, radEquator = loadGravFromFileToList(
        str(GGM03S_PATH), maxDeg=maxDegree
    )

    assert mu == muRef
    assert radEquator == radRef
    for degree in range(maxDegree + 1):
        np.testing.assert_allclose(cList[degree], cRef[degree], rtol=0.0, atol=0.0)
        np.testing.assert_allclose(sList[degree], sRef[degree], rtol=0.0, atol=0.0)


def testLoadGravFromFileToListRejectsDegreeAboveFileLimit():
    """Verify an out-of-range degree request raises a clear ValueError."""
    _, _, _, _, maxDegreeFile, maxOrderFile = _loadReferenceCoefficients(
        GGM03S_PATH, maxDegree=0
    )
    requestedDegree = min(maxDegreeFile, maxOrderFile) + 1

    with pytest.raises(ValueError, match="maximum degree/order"):
        loadGravFromFileToList(str(GGM03S_PATH), maxDeg=requestedDegree)


def testLoadGravFromFileToListRejectsNegativeDegree():
    """Verify negative requested degree raises a clear ValueError."""
    with pytest.raises(ValueError, match="must be non-negative"):
        loadGravFromFileToList(str(GGM03S_PATH), maxDeg=-1)


def testLoadGravFromFileToListUsesRowOrderColumn(tmpPath):
    """Verify rows are placed by explicit order value, even if row order is shuffled."""
    header, rows = _loadHeaderAndRows(GGM03S_PATH, maxDegree=2)
    rowsByIndex = {(int(row[0]), int(row[1])): row for row in rows}
    shuffledRows = [
        rowsByIndex[(0, 0)],
        rowsByIndex[(1, 0)],
        rowsByIndex[(1, 1)],
        rowsByIndex[(2, 0)],
        rowsByIndex[(2, 2)],
        rowsByIndex[(2, 1)],
    ]
    tempFile = _writeTempGravityFile(tmpPath, header, shuffledRows)

    cRef, sRef, _, _, _, _ = _loadReferenceCoefficients(GGM03S_PATH, maxDegree=2)
    cList, sList, _, _ = loadGravFromFileToList(str(tempFile), maxDeg=2)

    for degree in range(3):
        np.testing.assert_allclose(cList[degree], cRef[degree], rtol=0.0, atol=0.0)
        np.testing.assert_allclose(sList[degree], sRef[degree], rtol=0.0, atol=0.0)


def testLoadGravFromFileToListPreservesZeroForMissingOrder(tmpPath):
    """Verify missing coefficients remain zero at their degree/order index."""
    header, rows = _loadHeaderAndRows(GGM03S_PATH, maxDegree=2)
    rowsByIndex = {(int(row[0]), int(row[1])): row for row in rows}
    missingOrderRows = [
        rowsByIndex[(0, 0)],
        rowsByIndex[(1, 0)],
        rowsByIndex[(1, 1)],
        rowsByIndex[(2, 0)],
        rowsByIndex[(2, 2)],
    ]
    tempFile = _writeTempGravityFile(tmpPath, header, missingOrderRows)

    cRef, sRef, _, _, _, _ = _loadReferenceCoefficients(GGM03S_PATH, maxDegree=2)
    cList, sList, _, _ = loadGravFromFileToList(str(tempFile), maxDeg=2)

    assert cList[2][1] == 0.0
    assert sList[2][1] == 0.0
    assert cList[2][2] == cRef[2][2]
    assert sList[2][2] == sRef[2][2]


def testLoadGravFromFileToListRejectsDegreeRegression(tmpPath):
    """Verify decreasing degree rows are rejected."""
    header, rows = _loadHeaderAndRows(GGM03S_PATH, maxDegree=2)
    rowsByIndex = {(int(row[0]), int(row[1])): row for row in rows}
    regressedRows = [
        rowsByIndex[(0, 0)],
        rowsByIndex[(1, 0)],
        rowsByIndex[(2, 0)],
        rowsByIndex[(1, 1)],
    ]
    tempFile = _writeTempGravityFile(tmpPath, header, regressedRows)

    with pytest.raises(ValueError, match="non-decreasing degree"):
        loadGravFromFileToList(str(tempFile), maxDeg=2)


def testLoadPolyFromFileToListRejectsEmptyTabFile(tmpPath):
    """Verify empty .tab polyhedral files are rejected with a clear error."""
    emptyTabFile = tmpPath / "emptyShape.tab"
    emptyTabFile.write_text("")

    with pytest.raises(ValueError, match="polyhedral file is empty"):
        loadPolyFromFileToList(str(emptyTabFile))
