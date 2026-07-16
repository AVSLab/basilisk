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

r"""
Shared comparison-plot helpers for the dynamics-engine comparison series.

Each returned figure contains exactly one axis. Scalar comparisons produce one overlay figure and,
when MuJoCo data are available, one difference figure. Vector comparisons produce the same pair
for each component. Keeping these views in separate files makes each plot legible at publication
size and lets LaTeX subcaptions explain the relation between panels.
"""

import numpy as np
import matplotlib.pyplot as plt

import _comparisonValidation

# Paul Tol high-contrast palette used throughout the comparison paper.
COLOR_BSM = "#004488"
COLOR_MUJOCO = "#BB5566"
COLOR_DIFF = "#DDAA33"

_AXES = ("x", "y", "z")


def _plotBoth(ax, tBSM, yBSM, tMj, yMj, bsmLabel, mjLabel):
    """Overlay one BSM curve (thick translucent underlay) and one MuJoCo curve (thin) on ``ax``."""
    ax.plot(tBSM, yBSM, lw=4, alpha=0.4, color=COLOR_BSM, label=bsmLabel)
    if yMj is not None:
        ax.plot(tMj, yMj, lw=1.3, color=COLOR_MUJOCO, label=mjLabel)


def _difference(tBSM, yBSM, tMj, yMj):
    """Return BSM-minus-MuJoCo after requiring identical timestamp histories."""
    if yMj is None:
        return None, None
    _comparisonValidation.requireEqualLength(
        "plot difference", tBSM, yBSM, tMj, yMj)
    if not np.array_equal(np.asarray(tBSM), np.asarray(tMj)):
        raise ValueError("plot difference timestamp histories differ.")
    return np.asarray(tBSM), np.asarray(yBSM) - np.asarray(yMj)


def scalarComparison(figName, time, yBSM, yMj, ylabel, xlabel="Time [s]",
                     diffScale=1.0, diffUnit=None, bsmLabel="BSM", mjLabel="MuJoCo",
                     logDiff=False, figsize=(7.0, 2.6)):
    """Return separate one-axis figures for a scalar overlay and its difference.

    Args:
        figName (str): figure key returned in the ``{name: figure}`` mapping.
        time (numpy.ndarray): sample times [s] for the BSM curve.
        yBSM (numpy.ndarray): BSM scalar history, shape ``(N,)``.
        yMj (numpy.ndarray or None): MuJoCo scalar history, or None when MuJoCo is unavailable.
        ylabel (str): y-label for the overlay (top) axis.
        xlabel (str, optional): shared x-label. Defaults to ``"Time [s]"``.
        diffScale (float, optional): factor applied to the difference for display (e.g. 1e3 to show
            it in milli-units). Defaults to 1.0.
        diffUnit (str, optional): unit string for the difference axis; if None it is taken from the
            trailing bracketed unit of ``ylabel``.
        bsmLabel, mjLabel (str, optional): legend labels.
        logDiff (bool, optional): if True, plot ``|difference|`` on a log axis. Defaults to False.

    Returns:
        dict: figure-name to figure mapping. The difference key is ``figName+"_difference"``.
    """
    figures = {}
    fig, ax = plt.subplots(figsize=figsize, layout="constrained")
    _plotBoth(ax, time, yBSM, time, yMj, bsmLabel, mjLabel)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    if yMj is not None:
        ax.legend(loc="best")
    figures[figName] = fig

    tD, diff = _difference(time, yBSM, time, yMj)
    if diff is not None:
        fig, ax = plt.subplots(figsize=figsize, layout="constrained")
        if logDiff:
            ax.semilogy(tD, np.maximum(np.abs(diff)*diffScale, 1e-16), color=COLOR_DIFF)
        else:
            ax.plot(tD, diff*diffScale, color=COLOR_DIFF)
        unit = diffUnit if diffUnit is not None else _unitOf(ylabel)
        ax.set_ylabel("Difference" + (f" [{unit}]" if unit else ""))
        ax.set_xlabel(xlabel)
        figures[figName+"_difference"] = fig
    return figures


def componentComparison(figName, time, vBSM, vMj, quantity, unit, xlabel="Time [s]",
                        scale=1.0, bsmLabel="BSM", mjLabel="MuJoCo",
                        figsize=(4.8, 2.4)):
    """Return one-axis overlay and difference figures for each vector component.

    Args:
        figName (str): figure key.
        time (numpy.ndarray): sample times [s] for the BSM curves.
        vBSM (numpy.ndarray): BSM vector history, shape ``(N, 3)``.
        vMj (numpy.ndarray or None): MuJoCo vector history ``(M, 3)``, or None when unavailable.
        quantity (str): quantity name for the row labels, e.g. ``r"$\\omega_{BN}$"``.
        unit (str): unit string, e.g. ``"mrad/s"``.
        xlabel (str, optional): shared x-label. Defaults to ``"Time [s]"``.
        scale (float, optional): factor applied to both curves and the difference for display.
        bsmLabel, mjLabel (str, optional): legend labels.

    Returns:
        dict: mapping from component and difference names to one-axis figures.
    """
    figures = {}
    for col in range(3):
        componentName = figName+"_"+_AXES[col]
        fig, ax = plt.subplots(figsize=figsize, layout="constrained")
        _plotBoth(ax, time, vBSM[:, col]*scale, time,
                  None if vMj is None else vMj[:, col]*scale, bsmLabel, mjLabel)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(f"{quantity}$_{_AXES[col]}$ [{unit}]")
        if vMj is not None:
            ax.legend(loc="best", fontsize=8)
        figures[componentName] = fig

        tD, diff = _difference(time, vBSM[:, col], time, None if vMj is None else vMj[:, col])
        if diff is not None:
            fig, ax = plt.subplots(figsize=figsize, layout="constrained")
            ax.plot(tD, diff*scale, color=COLOR_DIFF)
            ax.set_xlabel(xlabel)
            ax.set_ylabel(f"Difference [{unit}]")
            figures[componentName+"Difference"] = fig
    return figures


def _unitOf(label):
    """Best-effort extraction of a trailing ``[unit]`` from an axis label, else empty string."""
    if "[" in label and label.rstrip().endswith("]"):
        return label[label.rindex("[") + 1:label.rindex("]")]
    return ""


def finalizeFigures(figureList):
    """Run each figure's constrained-layout solve before the caller closes the pyplot state.

    The scenarios return their figures and then call ``plt.close("all")`` so an interactive
    ``showPlots`` session does not leak windows. The unit test / doc build, however, saves those
    figures *after* that close, and a closed figure will not re-run its constrained-layout solve at
    ``savefig`` time -- which lets long y-labels overflow and clip. Drawing each figure here, while
    it is still live, bakes in the correct label positions so the later save is not clipped.

    Args:
        figureList (dict): mapping of figure name to matplotlib figure, as returned by a scenario.

    Returns:
        dict: the same ``figureList`` (returned for convenient chaining).
    """
    for fig in figureList.values():
        try:
            fig.canvas.draw()
        except Exception:
            pass
    return figureList
