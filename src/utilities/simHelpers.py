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

"""General-purpose simulation helpers.

These utilities are used by modules such as :ref:`simIncludeGravBody` and
:ref:`vizSupport`.  They are kept here so that importing those user-facing
modules does not pull in the test-only ``pytest`` dependency that
``unitTestSupport`` transitively would introduce.
"""

import errno
import math
import os
import shutil
import sys
from datetime import datetime, timedelta

import numpy as np

from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.architecture import bskUtilities
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile

bskPath = __path__[0]
_matplotlibDefaultsApplied = False


def _applyMatplotlibDefaults(mpl):
    """Apply Basilisk scenario plot defaults once."""
    global _matplotlibDefaultsApplied
    if _matplotlibDefaultsApplied:
        return

    mpl.rc("figure", facecolor="white")
    mpl.rc("xtick", labelsize=9)
    mpl.rc("ytick", labelsize=9)
    mpl.rc("figure", figsize=(5.75, 2.5))
    mpl.rc("axes", labelsize=10)
    mpl.rc("legend", fontsize=9)
    mpl.rc("figure", autolayout=True)
    mpl.rc("figure", max_open_warning=30)
    mpl.rc("legend", loc="lower right")
    _matplotlibDefaultsApplied = True


def _getPyplot():
    """Return ``matplotlib.pyplot`` after applying scenario plot defaults."""
    import matplotlib as mpl
    import matplotlib.pyplot as pyplot

    _applyMatplotlibDefaults(mpl)
    return pyplot


def _getTabulate():
    """Return Basilisk's LaTeX-friendly ``tabulate`` function."""
    from Basilisk.utilities import tabulate as tabulateModule

    for escapeKey in ("$", "\\", "_", "{", "}"):
        tabulateModule.LATEX_ESCAPE_RULES.pop(escapeKey, None)
    return tabulateModule.tabulate


if "matplotlib" in sys.modules:
    _applyMatplotlibDefaults(sys.modules["matplotlib"])


def EigenVector3d2np(eig):
    """convert Eigen vector3d to numpy"""
    return np.array([eig[0][0], eig[1][0], eig[2][0]])


def EigenVector3d2list(eig):
    """convert Eigen vector3d to list"""
    return EigenVector3d2np(eig).tolist()


def addTimeColumn(time, data):
    """Add a time column to the data set"""
    return np.transpose(np.vstack([[time], np.transpose(data)]))


def timeStringToGregorianUTCMsg(DateSpice, **kwargs):
    """convert a general time/date string to a gregoarian UTC msg object"""
    # set the data path
    if "dataPath" in kwargs:
        dataPath = kwargs["dataPath"]
        if not isinstance(dataPath, str):
            print("ERROR: dataPath must be a string argument")
            exit(1)
    else:
        dataPath = bskPath + "/supportData/EphemerisData/"  # default value

    # load spice kernel and convert the string into a UTC date/time string
    naif0012_path = get_path(DataFile.EphemerisData.naif0012)
    pyswice.furnsh_c(str(naif0012_path))
    et = pyswice.new_doubleArray(1)
    pyswice.str2et_c(DateSpice, et)
    etEpoch = pyswice.doubleArray_getitem(et, 0)
    ep1 = pyswice.et2utc_c(etEpoch, "C", 6, 255, "Yo")
    pyswice.unload_c(str(naif0012_path))

    try:
        # convert UTC string to datetime object
        datetime_object = datetime.strptime(ep1, "%Y %b %d %H:%M:%S.%f")

        # Validate month is in range 1-12
        if datetime_object.month < 1 or datetime_object.month > 12:
            raise ValueError(f"Invalid month value: {datetime_object.month}")

        # populate the epochMsg with the gregorian UTC date/time information
        epochMsgStructure = messaging.EpochMsgPayload()
        epochMsgStructure.year = datetime_object.year
        epochMsgStructure.month = datetime_object.month
        epochMsgStructure.day = datetime_object.day
        epochMsgStructure.hours = datetime_object.hour
        epochMsgStructure.minutes = datetime_object.minute
        epochMsgStructure.seconds = (
            datetime_object.second + datetime_object.microsecond / 1e6
        )

        epochMsg = messaging.EpochMsg().write(epochMsgStructure)

        # Store the message in a global registry to prevent garbage collection
        if not hasattr(timeStringToGregorianUTCMsg, "_msg_registry"):
            timeStringToGregorianUTCMsg._msg_registry = []
        timeStringToGregorianUTCMsg._msg_registry.append(epochMsg)

        return epochMsg

    except Exception as e:
        print(f"Error processing date string '{ep1}': {str(e)}")
        print(f"Original input string was: {DateSpice}")
        raise


def writeTableLaTeX(tableName, tableHeaders, caption, array, path):
    """Take a list and return equivalent LaTeX table code."""

    texFileName = path + "/../_Documentation/AutoTeX/" + tableName + ".tex"

    if not os.path.exists(os.path.dirname(texFileName)):
        try:
            os.makedirs(os.path.dirname(texFileName))
        except OSError as exc:
            if exc.errno != errno.EEXIST:
                raise
    with open(texFileName, "w") as texTable:
        table = _getTabulate()(array, tableHeaders, tablefmt="latex", numalign="center")

        texTable.write(r"\begin{table}[htbp]")
        texTable.write(r"\caption{" + caption + "}")
        texTable.write(r"\label{tbl:" + tableName + "}")
        texTable.write(r"\centering")
        texTable.write(table)
        texTable.write(r"\end{table}")


def writeTeXSnippet(snippetName, texSnippet, path):
    """Write a LaTeX snippet to a file."""

    texFileName = path + "/../_Documentation/AutoTeX/" + snippetName + ".tex"

    if not os.path.exists(os.path.dirname(texFileName)):
        try:
            os.makedirs(os.path.dirname(texFileName))
        except OSError as exc:
            if exc.errno != errno.EEXIST:
                raise
    with open(texFileName, "w") as fileHandler:
        fileHandler.write(texSnippet)


def getScenarioFigureFileName(figureName, path, extension=".svg"):
    """Return the documentation image path for a scenario figure."""
    if not extension.startswith("."):
        extension = "." + extension

    searchPath = os.path.abspath(path)
    if os.path.isfile(searchPath):
        searchPath = os.path.dirname(searchPath)

    while True:
        if os.path.isdir(os.path.join(searchPath, "docs", "source")):
            scenarioFigurePath = os.path.join(
                searchPath,
                "docs",
                "source",
                "_images",
                "Scenarios",
            )
            return os.path.join(scenarioFigurePath, figureName + extension)

        parentPath = os.path.dirname(searchPath)
        if parentPath == searchPath:
            scenarioFigurePath = os.path.join(
                path,
                "..",
                "..",
                "docs",
                "source",
                "_images",
                "Scenarios",
            )
            return os.path.abspath(
                os.path.join(scenarioFigurePath, figureName + extension)
            )

        searchPath = parentPath


def _createScenarioFigurePath(imgFileName):
    """Create the documentation image folder if needed."""
    if not os.path.exists(os.path.dirname(imgFileName)):
        try:
            os.makedirs(os.path.dirname(imgFileName))
        except OSError as exc:
            if exc.errno != errno.EEXIST:
                raise


def saveScenarioFigure(figureName, plt, path, extension=".svg"):
    """Save a Python scenario result into the documentation image folder."""
    imgFileName = getScenarioFigureFileName(figureName, path, extension)
    _createScenarioFigurePath(imgFileName)
    plt.savefig(imgFileName, transparent=True)
    try:
        _getPyplot().close(plt)
    except Exception:
        try:
            plt.close()
        except Exception:
            pass


def saveScenarioGraphvizFigure(
    figureName,
    simulationBase,
    path,
    extension=".svg",
    show_plots=False,
    **kwargs
):
    """Save a Graphviz message flow figure into the documentation image folder."""
    if shutil.which("dot") is None:
        return None

    imgFileName = getScenarioFigureFileName(figureName, path, extension)
    graphvizFormat = extension.lstrip(".")
    _createScenarioFigurePath(imgFileName)
    return simulationBase.ShowMessageConnectionFigure(
        renderer="graphviz",
        fileName=imgFileName,
        show_plots=show_plots,
        graphvizFormat=graphvizFormat,
        **kwargs
    )


def saveFigurePDF(figureName, plt, path):
    """Save a figure as a PDF."""
    figFileName = os.path.join(path, figureName + ".pdf")
    if not os.path.exists(os.path.dirname(figFileName)):
        try:
            os.makedirs(os.path.dirname(figFileName))
        except OSError as exc:
            if exc.errno != errno.EEXIST:
                raise
    plt.savefig(figFileName, transparent=True, pad_inches=0.05)


def writeFigureLaTeX(figureName, caption, plt, format, path):
    """Save a figure and associated TeX code snippet."""
    texFileName = os.path.join(
        path, "..", "_Documentation", "AutoTeX", figureName + ".tex"
    )
    if not os.path.exists(os.path.dirname(texFileName)):
        try:
            os.makedirs(os.path.dirname(texFileName))
        except OSError as exc:
            if exc.errno != errno.EEXIST:
                raise
    with open(texFileName, "w") as texFigure:
        texFigure.write(r"\begin{figure}[htbp]")
        texFigure.write(r"\centerline{")
        texFigure.write(
            r"\includegraphics[" + format + "]{AutoTeX/" + figureName + r"}}"
        )
        texFigure.write(r"\caption{" + caption + r"}")
        texFigure.write(r"\label{fig:" + figureName + r"}")
        texFigure.write(r"\end{figure}")

        texFileName = path + "/../_Documentation/AutoTeX/" + figureName + ".pdf"
        plt.savefig(texFileName, transparent=True)


def getLineColor(idx, maxNum):
    """Pick a color from a scenario plotting color map."""
    import matplotlib as mpl
    import matplotlib.cm as cmx
    import matplotlib.colors as colors
    import matplotlib.pyplot as pyplot

    _applyMatplotlibDefaults(mpl)
    values = list(range(0, maxNum + 2))
    colorMap = pyplot.get_cmap("gist_earth")
    cNorm = colors.Normalize(vmin=0, vmax=values[-1])
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=colorMap)
    return scalarMap.to_rgba(values[idx + 1])


def np2EigenMatrix3d(mat):
    """Convert a 3D NumPy matrix to an Eigen matrix."""
    return [
        [mat[0], mat[1], mat[2]],
        [mat[3], mat[4], mat[5]],
        [mat[6], mat[7], mat[8]],
    ]


def np2EigenVectorXd(vec):
    """Convert a NumPy vector to an Eigen vector."""
    npVec = []
    for item in vec:
        npVec.extend([[item]])

    return npVec


def npList2EigenXdVector(list):
    """Convert a list of arrays to a list of Eigen values."""
    eigenList = bskUtilities.Eigen3dVector()
    for pos in list:
        eigenList.push_back(pos)
    return eigenList


def flattenList(matrix):
    """Return a flattened list.

    Args:
        matrix: List of lists.

    Returns:
        Flattened list.
    """
    flatList = []
    for row in matrix:
        flatList.extend(row)
    return flatList


def pullVectorSetFromData(inpMat):
    """Extract vector data from a matrix whose first column is time data."""
    outMat = np.array(inpMat).transpose()
    return outMat[1:].transpose()


def decimalYearToDateTime(start):
    """Convert a decimal year to a :class:`datetime.datetime` object."""
    year = int(start)
    rem = start - year

    base = datetime(year, 1, 1)
    return base + timedelta(
        seconds=(base.replace(year=base.year + 1) - base).total_seconds() * rem
    )


def columnToRowList(set):
    """Loop through a column list and return a row list."""
    ans = []
    for item in set:
        ans.append(item[0])
    return ans


def checkMethodKeyword(karglist, kwargs):
    """Check that keyword arguments are in the allowed keyword list."""
    for key in kwargs:
        if key not in karglist:
            print(
                "ERROR: you tried to use an incorrect keyword "
                + key
                + ". Options include:"
            )
            print(karglist)
            exit(1)


def removeTimeFromData(dataList):
    """Remove the time column from a data list."""
    return (dataList.transpose()[1 : len(dataList[0])]).transpose()


def samplingTime(simTime, baseTimeStep, numDataPoints):
    """Return a sample time that approximates a target number of samples.

    Args:
        simTime: [ns] Total simulation duration.
        baseTimeStep: [ns] Baseline sampling period.
        numDataPoints: Nominal desired number of data points over the
            simulation duration.

    Returns:
        [ns] Sampling period.
    """
    deltaTime = math.floor(simTime / baseTimeStep / (numDataPoints - 1)) * baseTimeStep
    if deltaTime < 1:
        deltaTime = 1  # [ns]
    return deltaTime
