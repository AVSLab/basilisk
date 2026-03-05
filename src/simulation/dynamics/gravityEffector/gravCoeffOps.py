#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
import csv

from Basilisk import __path__

def loadGravFromFile(
        fileName: str,
        spherHarm: "SphericalHarmonicsGravityModel",
        maxDeg: int = 2
    ):
    """Load spherical-harmonic gravity coefficients into a model object.

    This is a thin convenience wrapper around :func:`loadGravFromFileToList`
    that parses the file and writes the resulting values into the provided
    ``SphericalHarmonicsGravityModel`` instance.

    :param fileName: Path to a JPL-format spherical harmonics CSV file.
    :param spherHarm: Gravity model object to populate.
    :param maxDeg: Maximum degree/order to load.
    :raises ValueError: If the file format or requested degree is invalid.
    """

    [clmList, slmList, mu, radEquator] = loadGravFromFileToList(fileName, maxDeg=maxDeg)

    spherHarm.muBody = mu
    spherHarm.radEquator = radEquator
    spherHarm.cBar = clmList
    spherHarm.sBar = slmList
    spherHarm.maxDeg = maxDeg

def loadGravFromFileToList(fileName: str, maxDeg: int = 2):
    """Parse spherical-harmonic coefficients from a JPL-format CSV file.

    The first row is expected to contain metadata in the canonical JPL format:
    equatorial radius, gravitational parameter, uncertainty in ``mu`` (unused),
    maximum degree, maximum order, normalization flag, reference longitude, and
    reference latitude.

    Coefficients are returned as triangular degree/order lists where each row
    has length ``degree + 1``. Rows in the file must be ordered by
    non-decreasing degree; duplicate degree/order pairs are rejected.

    :param fileName: Path to a JPL-format spherical harmonics CSV file.
    :param maxDeg: Maximum degree/order to load.
    :return: ``[clmList, slmList, mu, radEquator]``.
    :raises ValueError: If input arguments or file contents are invalid.
    """
    if maxDeg < 0:
        raise ValueError(
            f"Requested using Spherical Harmonics of degree {maxDeg}, but the degree"
            " must be non-negative"
        )

    with open(fileName, 'r') as csvfile:
        gravReader = csv.reader(csvfile, delimiter=',')
        firstRow = next(gravReader)

        try:
            radEquator = float(firstRow[0])
            mu = float(firstRow[1])
            # firstRow[2] is uncertainty in mu, not needed for Basilisk
            maxDegreeFile = int(firstRow[3])
            maxOrderFile = int(firstRow[4])
            coefficientsNormalized = int(firstRow[5]) == 1
            refLong = float(firstRow[6])
            refLat = float(firstRow[7])
        except Exception as ex:
            raise ValueError("File is not in the expected JPL format for "
                             "spherical Harmonics") from ex

        if maxDegreeFile < maxDeg or maxOrderFile < maxDeg:
            raise ValueError(f"Requested using Spherical Harmonics of degree {maxDeg}"
                             f", but file '{fileName}' has maximum degree/order of"
                             f"{min(maxDegreeFile, maxOrderFile)}")

        if not coefficientsNormalized:
            raise ValueError("Coefficients in given file are not normalized. This is "
                            "not currently supported in Basilisk.")

        if refLong != 0 or refLat != 0:
            raise ValueError("Coefficients in given file use a reference longitude"
                             " or latitude that is not zero. This is not currently "
                             "supported in Basilisk.")

        # Allocate a lower-triangular storage layout:
        # degree d stores coefficients for orders m in [0, d].
        clmList = [[0.0] * (degree + 1) for degree in range(maxDeg + 1)]
        slmList = [[0.0] * (degree + 1) for degree in range(maxDeg + 1)]
        assignedCoefficients = set()
        previousDegree = -1
        for gravRow in gravReader:
            try:
                rowDeg = int(gravRow[0])
                rowOrder = int(gravRow[1])
                clm = float(gravRow[2])
                slm = float(gravRow[3])
            except Exception as ex:
                raise ValueError("Gravity coefficient row is not in the expected format") from ex

            if rowDeg < 0 or rowOrder < 0 or rowOrder > rowDeg:
                raise ValueError(
                    f"Invalid gravity coefficient row with degree/order ({rowDeg}, {rowOrder})"
                )

            # Enforce monotonic degree ordering so malformed/shuffled files do
            # not silently produce ambiguous coefficient tables.
            if rowDeg < previousDegree:
                raise ValueError(
                    "Gravity coefficient rows must be ordered by non-decreasing degree"
                )
            previousDegree = rowDeg

            # Input files are usually ordered by increasing degree; stop once
            # we passed the requested truncation degree.
            if rowDeg > maxDeg:
                break

            coeffIndex = (rowDeg, rowOrder)
            if coeffIndex in assignedCoefficients:
                raise ValueError(
                    f"Duplicate gravity coefficient row for degree/order ({rowDeg}, {rowOrder})"
                )
            assignedCoefficients.add(coeffIndex)

            clmList[rowDeg][rowOrder] = clm
            slmList[rowDeg][rowOrder] = slm

        return [clmList, slmList, mu, radEquator]


def loadPolyFromFile(fileName: str, poly: "PolyhedralGravityModel"):
    """Load polyhedral shape data into a model object.

    This is a thin convenience wrapper around :func:`loadPolyFromFileToList`
    that parses supported polyhedron files and writes vertices/facets into the
    provided ``PolyhedralGravityModel`` instance.

    :param fileName: Path to a supported polyhedron file (``.tab``, ``.obj``,
        or ``.txt``).
    :param poly: Polyhedral gravity model object to populate.
    :raises ValueError: If the file extension or contents are invalid.
    """
    [vertList, faceList, _, _] = loadPolyFromFileToList(fileName)
    poly.xyzVertex = vertList
    poly.orderFacet = faceList

def loadPolyFromFileToList(fileName: str):
    """Parse a polyhedral shape model from disk.

    Supported formats are:

    - ``.tab``: Gaskell ``(nVertex nFacet + data rows)`` or PDS3/OBJ-like text.
    - ``.obj``: Wavefront-style ``v``/``f`` entries.
    - ``.txt``: Legacy format with ``nVertex nFacet`` header followed by rows.

    Vertex positions are converted from kilometers in file inputs to meters in
    output lists. Facet index conventions are preserved according to each file
    format's expected indexing.

    :param fileName: Path to a supported polyhedron file.
    :return: ``[vertList, faceList, nVertex, nFacet]``.
    :raises ValueError: If the file extension is not supported or file content
        cannot be parsed.
    """
    with open(fileName) as polyFile:
        if fileName.endswith('.tab'):
            try:
                firstLine = next(polyFile)  # read first line
            except StopIteration as ex:
                raise ValueError("The '.tab' polyhedral file is empty.") from ex

            try:
                # Gaskell .tab starts with "<nVertex> <nFacet>".
                nVertex, nFacet = [int(x) for x in firstLine.split()]
                fileType = 'gaskell'
            except ValueError:
                # PDS3 .tab stores explicit "v"/"f" prefixed rows.
                polyFile.seek(0)
                fileType = 'pds3'

            if fileType == 'gaskell':
                vertList = []
                faceList = []

                contLines = 0
                for line in polyFile:
                    arrtemp = []

                    for x in line.split():
                        arrtemp.append(float(x))

                    if contLines < nVertex:
                        # Input vertex coordinates are in km; Basilisk uses m.
                        vertList.append([float(arrtemp[1]*1e3),float(arrtemp[2]*1e3),float(arrtemp[3]*1e3)])
                    else:
                        # Facet indices are already in the expected convention.
                        faceList.append([int(arrtemp[1]),int(arrtemp[2]),int(arrtemp[3])])

                    contLines += 1
            elif fileType == 'pds3':
                nVertex = 0
                nFacet = 0
                vertList = []
                faceList = []
                for line in polyFile:
                    arrtemp = line.split()
                    if arrtemp:
                        if arrtemp[0] == 'v':
                            nVertex += 1
                            # Input vertex coordinates are in km; Basilisk uses m.
                            vertList.append([float(arrtemp[1])*1e3, float(arrtemp[2])*1e3, float(arrtemp[3])*1e3])
                        elif arrtemp[0] == 'f':
                            nFacet += 1
                            # Convert zero-based facet indices to one-based.
                            faceList.append([int(arrtemp[1])+1, int(arrtemp[2])+1, int(arrtemp[3])+1])
        elif fileName.endswith('.obj'):
            nVertex = 0
            nFacet = 0
            vertList = []
            faceList = []
            for line in polyFile:
                arrtemp = line.split()
                if arrtemp:
                    if arrtemp[0] == 'v':
                        nVertex += 1
                        # Input vertex coordinates are in km; Basilisk uses m.
                        vertList.append([float(arrtemp[1])*1e3, float(arrtemp[2])*1e3, float(arrtemp[3])*1e3])
                    elif arrtemp[0] == 'f':
                        nFacet += 1
                        # OBJ facet indices are expected one-based.
                        faceList.append([int(arrtemp[1]), int(arrtemp[2]), int(arrtemp[3])])
        elif fileName.endswith('.txt'):
            nVertex, nFacet = [int(x) for x in next(polyFile).split()] # read first line
            vertList = []
            faceList = []

            contLines = 0
            for line in polyFile:
                arrtemp = []

                for x in line.split():
                    arrtemp.append(float(x))

                if contLines < nVertex:
                    # Input vertex coordinates are in km; Basilisk uses m.
                    vertList.append([float(arrtemp[0]*1e3),float(arrtemp[1]*1e3),float(arrtemp[2]*1e3)])
                else:
                    faceList.append([int(arrtemp[0]),int(arrtemp[1]),int(arrtemp[2])])

                contLines += 1
        else:
            raise ValueError("Unrecognized file extension. Valid extensions are "
                             "'.tab', '.obj', and '.txt'")

        return [vertList, faceList, nVertex, nFacet]
