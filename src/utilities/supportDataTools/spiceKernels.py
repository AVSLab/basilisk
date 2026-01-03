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

from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile

DEFAULT_KERNELS = (
    DataFile.EphemerisData.naif0012,
    DataFile.EphemerisData.pck00010,
    DataFile.EphemerisData.de_403_masses,
    DataFile.EphemerisData.de430,
)


def configure_spiceinterface_default_kernels(spice_obj):
    # Trigger pooch fetch
    paths = [
        str(get_path(DataFile.EphemerisData.naif0012)),
        str(get_path(DataFile.EphemerisData.pck00010)),
        str(get_path(DataFile.EphemerisData.de_403_masses)),
        str(get_path(DataFile.EphemerisData.de430)),
    ]

    # Feed explicit kernel paths into SpiceInterface
    spice_obj.clearKernelPaths()
    spice_obj.addKernelPaths(paths)
