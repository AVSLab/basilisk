#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

from enum import Enum
from pathlib import Path
import functools
import requests
import pooch
import logging
from typing import Optional

from Basilisk.utilities.supportDataTools.registrySnippet import REGISTRY
from Basilisk import __version__

pooch_logger = pooch.utils.get_logger()
pooch_logger.setLevel(logging.INFO)

# Override URLs for large NAIF kernels (not in GitHub repo)
EXTERNAL_KERNEL_URLS = {
    "supportData/EphemerisData/de430.bsp": "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de430.bsp",
    "supportData/EphemerisData/naif0012.tls": "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/lsk/naif0012.tls",
    "supportData/EphemerisData/pck00010.tpc": "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/pck00010.tpc",
    "supportData/EphemerisData/de-403-masses.tpc": "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/de-403-masses.tpc",
    "supportData/EphemerisData/hst_edited.bsp": "https://naif.jpl.nasa.gov/pub/naif/HST/kernels/spk/hst_edited.bsp",
    "supportData/EphemerisData/nh_pred_od077.bsp": "https://naif.jpl.nasa.gov/pub/naif/pds/data/nh-j_p_ss-spice-6-v1.0/nhsp_1000/data/spk/nh_pred_od077.bsp",
    "supportData/SkyBrightnessData/haslam408_dsds_Remazeilles2014.fits": "https://lambda.gsfc.nasa.gov/data/foregrounds/haslam_2014/haslam408_dsds_Remazeilles2014.fits",
}

# Do not set hashes for files fetched from external URLs such as JPL NAIF as
# they may change without notice.
for key in EXTERNAL_KERNEL_URLS:
    REGISTRY[key] = None

DATA_VERSION = f"v{__version__}"

ALBEDO_DATA_BASE_PATH = "AlbedoData/"
ATMOSPHERE_DATA_BASE_PATH = "AtmosphereData/"
DENTON_GEO_BASE_PATH = "DentonGEO/"
EPHEMERIS_DATA_BASE_PATH = "EphemerisData/"
LOCAL_GRAV_DATA_BASE_PATH = "LocalGravData/"
MAGNETIC_FIELD_BASE_PATH = "MagneticField/"
SKY_BRIGHTNESS_BASE_PATH = "SkyBrightnessData/"


@functools.lru_cache(maxsize=32)
def tag_exists(tag_url: str) -> bool:
    """Return True if the given GitHub tag URL exists.

    Cached so repeated calls never trigger additional network requests.
    """
    try:
        r = requests.head(tag_url, timeout=1)
        return r.status_code == 200
    except Exception:
        return False


def find_local_support_data() -> Optional[Path]:
    """
    Return the path to the local ``supportData`` directory if running
    from a cloned repo, otherwise return ``None``.

    Works whether running from source (src/) or from built modules (dist3/Basilisk/).
    """
    module_path = Path(__file__).resolve()
    # Walk up the directory tree looking for supportData. From src/ it's 4
    # levels up, from dist3/Basilisk/ it's 5 levels up.
    for parent in list(module_path.parents)[:6]:
        support_data = parent / "supportData"
        if support_data.is_dir():
            return support_data
    return None


# Compute the base GitHub URL once at import time.
LOCAL_SUPPORT = find_local_support_data()

# For remote fetches (wheel installs), check if the version tag exists. Fall
# back to develop if not
_version_tag_url = f"https://github.com/AVSLab/basilisk/releases/tag/{DATA_VERSION}"
_remote_version = DATA_VERSION if tag_exists(_version_tag_url) else "develop"
BASE_URL = f"https://raw.githubusercontent.com/AVSLab/basilisk/{_remote_version}/"

POOCH = pooch.create(
    path=pooch.os_cache("bsk_support_data"),
    base_url=BASE_URL,
    registry=REGISTRY,
    urls=EXTERNAL_KERNEL_URLS,
)


def _local_rel(rel: str) -> str:
    """Convert a registry-style key into a path relative to LOCAL_SUPPORT."""
    prefix = "supportData/"
    return rel[len(prefix) :] if rel.startswith(prefix) else rel


def get_path(file_enum: Enum) -> Path:
    """
    Return a filesystem path for the requested supportData file.

    If running from a local Basilisk repo, the file is returned directly from
    ``supportData/``. Otherwise, it is fetched or retrieved from the Pooch
    cache.
    """
    rel = relpath(file_enum)

    if LOCAL_SUPPORT:
        local = LOCAL_SUPPORT / _local_rel(rel)
        if local.exists():
            return local

        # When running locally, allow remote fetch for external URLs like the
        # large NAIF kernels not included in the repo.
        if rel in EXTERNAL_KERNEL_URLS:
            return Path(POOCH.fetch(rel))

        raise FileNotFoundError(f"Support data file not found in local repo: {local}")

    # No local repo - fetch from remote (installed from wheel)
    try:
        return Path(POOCH.fetch(rel))
    except Exception as e:
        raise FileNotFoundError(f"Support data file not found via pooch: {rel}") from e


class DataFile:
    class AlbedoData(Enum):
        Earth_ALB_2018_CERES_All_1x1 = "Earth_ALB_2018_CERES_All_1x1.csv"
        Earth_ALB_2018_CERES_All_5x5 = "Earth_ALB_2018_CERES_All_5x5.csv"
        Earth_ALB_2018_CERES_All_10x10 = "Earth_ALB_2018_CERES_All_10x10.csv"
        Earth_ALB_2018_CERES_Clear_1x1 = "Earth_ALB_2018_CERES_Clear_1x1.csv"
        Earth_ALB_2018_CERES_Clear_5x5 = "Earth_ALB_2018_CERES_Clear_5x5.csv"
        Earth_ALB_2018_CERES_Clear_10x10 = "Earth_ALB_2018_CERES_Clear_10x10.csv"
        earthReflectivityMean_1p25x1 = "earthReflectivityMean_1p25x1.dat"
        earthReflectivityMean_5x5 = "earthReflectivityMean_5x5.dat"
        earthReflectivityMean_10x10 = "earthReflectivityMean_10x10.dat"
        earthReflectivityStd_1p25x1 = "earthReflectivityStd_1p25x1.dat"
        earthReflectivityStd_5x5 = "earthReflectivityStd_5x5.dat"
        earthReflectivityStd_10x10 = "earthReflectivityStd_10x10.dat"
        Mars_ALB_TES_1x1 = "Mars_ALB_TES_1x1.csv"
        Mars_ALB_TES_5x5 = "Mars_ALB_TES_5x5.csv"
        Mars_ALB_TES_10x10 = "Mars_ALB_TES_10x10.csv"
        marsReflectivityMean_1p25x1 = "marsReflectivityMean_1p25x1.dat"
        marsReflectivityMean_5x5 = "marsReflectivityMean_5x5.dat"
        marsReflectivityMean_10x10 = "marsReflectivityMean_10x10.dat"

    class AtmosphereData(Enum):
        # You’re currently missing these three in the enum but they’re in REGISTRY:
        EarthGRAMNominal = "EarthGRAMNominal.txt"
        MarsGRAMNominal = "MarsGRAMNominal.txt"
        NRLMSISE00Nominal = "NRLMSISE00Nominal.txt"

        JupiterGRAMNominal = "JupiterGRAMNominal.csv"
        NeptuneGRAMNominal = "NeptuneGRAMNominal.csv"
        TitanGRAMNominal = "TitanGRAMNominal.csv"
        UranusGRAMNominal = "UranusGRAMNominal.csv"
        USStandardAtmosphere1976 = "USStandardAtmosphere1976.csv"
        VenusGRAMNominal = "VenusGRAMNominal.csv"

    class DentonGEOData(Enum):
        model_e_array_all = "model_e_array_all.txt"
        model_e_array_high = "model_e_array_high.txt"
        model_e_array_low = "model_e_array_low.txt"
        model_e_array_mid = "model_e_array_mid.txt"
        model_i_array_all = "model_i_array_all.txt"
        model_i_array_high = "model_i_array_high.txt"
        model_i_array_low = "model_i_array_low.txt"
        model_i_array_mid = "model_i_array_mid.txt"

    class EphemerisData(Enum):
        de_403_masses = "de-403-masses.tpc"
        de430 = "de430.bsp"
        hst_edited = "hst_edited.bsp"
        MVN_SCLKSCET_00000 = "MVN_SCLKSCET.00000.tsc"
        naif0011 = "naif0011.tls"
        naif0012 = "naif0012.tls"
        nh_pred_od077 = "nh_pred_od077.bsp"
        pck00010 = "pck00010.tpc"

    class LocalGravData(Enum):
        eros007790 = "eros007790.tab"
        GGM2BData = "GGM2BData.txt"
        GGM03S_J2_only = "GGM03S-J2-only.txt"
        GGM03S = "GGM03S.txt"
        VESTA20H = "VESTA20H.txt"

    class MagneticFieldData(Enum):
        WMM = "WMM.COF"

    class SkyBrightnessData(Enum):
        skyTemperature408MHz = "haslam408_dsds_Remazeilles2014.fits"


CATEGORY_BASE_PATHS = {
    "AlbedoData": ALBEDO_DATA_BASE_PATH,
    "AtmosphereData": ATMOSPHERE_DATA_BASE_PATH,
    "DentonGEOData": DENTON_GEO_BASE_PATH,
    "EphemerisData": EPHEMERIS_DATA_BASE_PATH,
    "LocalGravData": LOCAL_GRAV_DATA_BASE_PATH,
    "MagneticFieldData": MAGNETIC_FIELD_BASE_PATH,
    "SkyBrightnessData": SKY_BRIGHTNESS_BASE_PATH,
}


def relpath(file_enum: Enum) -> str:
    category_name = type(file_enum).__name__
    try:
        base = CATEGORY_BASE_PATHS[category_name]
        rel = base + file_enum.value
        return "supportData/" + rel
    except KeyError:
        raise ValueError(f"Unknown supportData category: {category_name}")
