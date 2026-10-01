#!/usr/bin/env python3
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

"""Require combined libraries during the temporary cross-platform CI experiment."""

import importlib
import importlib.util
from pathlib import Path

import Basilisk


GROUPS = (
    (None, "Basilisk.fswAlgorithms._fswCoreNative", "Basilisk.fswAlgorithms._mrpFeedback"),
    (None, "Basilisk.simulation._simulationCoreNative", "Basilisk.simulation._spacecraft"),
    ("mujoco", "Basilisk.simulation._mujocoNative", "Basilisk.simulation._thrOnTimeToForce"),
    ("opNav", "Basilisk._opNavNative", "Basilisk.simulation._camera"),
)


def main():
    """Verify native module origins and the absence of disabled optional groups."""
    for feature, library, representative in GROUPS:
        expected = feature is None or Basilisk.hasBuildFeature(feature)
        spec = importlib.util.find_spec(library)
        if not expected:
            if spec is not None:
                raise SystemExit(f"Unexpected disabled binding group: {library}")
            print(f"OK disabled group: {library}", flush=True)
            continue
        if spec is None:
            raise SystemExit(f"Missing combined binding group: {library}")
        native = importlib.import_module(representative)
        if Path(native.__file__).resolve() != Path(spec.origin).resolve():
            raise SystemExit(f"{representative} loaded {native.__file__}, expected {spec.origin}")
        print(f"OK combined binding: {representative} -> {native.__file__}", flush=True)


if __name__ == "__main__":
    main()
