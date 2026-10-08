#
#  ISC License
#
#  Copyright (c) 2026, PIC4SeR & AVS Lab, Politecnico di Torino & Argotec S.R.L., University of Colorado Boulder
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

"""Fast checks of the GMAT and Orekit reference generators for differences between machines: operating system file
layouts, path separators and escaping, line endings and tool releases.

The GMAT ``README.txt`` files of several releases are fetched once with Pooch (cached like the Basilisk support data,
see ``BSK_SUPPORT_DATA_CACHE``) and the tests are skipped when they cannot be downloaded. Neither GMAT nor Orekit
is run.
The test of the Orekit jar needs ``orekit_jpype`` and a Java runtime, which the CI runners do not have: it is marked
``ciSkip`` so that the CI runs deselect it (a skip would be an error there) and it skips itself when they are missing.
"""

import os
import sys
from pathlib import Path

import pytest

ACCURACY_DIR = Path(__file__).resolve().parents[1] / "accuracyComparison"
sys.path.insert(0, str(ACCURACY_DIR))

import comparisonCommon as common  # noqa: E402
import generate_gmat_reference as gmat  # noqa: E402

# GMAT releases whose ``README.txt`` states the release in its copyright notice: file name -> SHA-256
GMAT_READMES = {
    "R2020a": "1f0960b306cdb10d36239f105f0b68ec80c90202b7498a1a01bc3a1058491966",
    "R2022a": "5ad0bf193191d60fb85ef498d4cea42ef0f9532447b5557dab992149b546e471",
    "R2025a": "5d0736ac2b32cad9a667ebb16f7dd0f188fdb823f74c8bf57e12e84b0abbaa5c",
    "R2026a": "e3e498dbb5d23560a6ac8c6ead8563056e9b9b1f932fa4ff29ee3742ff8bd655",
}
GMAT_README_URL = "https://sourceforge.net/projects/gmat/files/GMAT/GMAT-{release}/README.txt/download"

# Files in ``bin/`` of the distribution of each platform (the Windows files are unversioned, the Linux ones are not)
PLATFORM_LAYOUTS = {
    "linux": ["GmatConsole", "libGmatBase.so", "libGmatBase.so.{release}"],
    "macos": ["GmatConsole", "GMAT-{release}_Beta.app"],
    "windows": ["GMAT.exe", "GmatConsole.exe", "libGmatBase.dll", "libGmatUtil.dll"],
}
README_TEXT = b"NASA Docket No. GSC-19468-1, identified as GMAT Version R2026a\n"
BANNER = "General Mission Analysis Tool\nConsole Based Version\nBuild Date: Mar 30 2026  10:09:27\n"


@pytest.fixture(scope="module")
def gmatReadmes():
    """Return the paths of the cached GMAT ``README.txt`` files, skipping the tests when they cannot be fetched."""
    pooch = pytest.importorskip("pooch")
    cache = os.environ.get("BSK_SUPPORT_DATA_CACHE")
    cachePath = Path(cache) if cache else Path(pooch.os_cache("bsk_support_data"))
    # Pooch creates the folder with a check-then-create that fails when parallel xdist workers race on a cold cache
    (cachePath / "GMAT").mkdir(parents=True, exist_ok=True)
    fetcher = pooch.create(path=cachePath, base_url="",
                           retry_if_failed=2,
                           registry={f"GMAT/README_{r}.txt": f"sha256:{h}" for r, h in GMAT_READMES.items()},
                           urls={f"GMAT/README_{r}.txt": GMAT_README_URL.format(release=r) for r in GMAT_READMES})
    try:
        return {r: Path(fetcher.fetch(f"GMAT/README_{r}.txt", downloader=pooch.HTTPDownloader(timeout=30)))
                for r in GMAT_READMES}
    except Exception as error:  # offline, mirror down, or file changed upstream
        pytest.skip(f"GMAT README files unavailable: {error}")


def _installation(root, platform, release, readmeText):
    """Create the file layout of a GMAT distribution of ``platform``; return the console path."""
    binDir = root / "bin"
    binDir.mkdir(parents=True)
    for name in PLATFORM_LAYOUTS[platform]:
        (binDir / name.format(release=release)).touch()
    (root / "README.txt").write_bytes(readmeText)
    return gmat.gmatConsole(root)


@pytest.mark.parametrize("lineEnding", [b"\n", b"\r\n"], ids=["lf", "crlf"])
@pytest.mark.parametrize("platform", sorted(PLATFORM_LAYOUTS))
@pytest.mark.parametrize("release", sorted(GMAT_READMES))
def test_gmat_release_is_detected_for_every_release_platform_and_line_ending(
        tmp_path, monkeypatch, gmatReadmes, release, platform, lineEnding):
    """Verify the release of each GMAT version is read from the real README for the layout of each platform."""
    readme = gmatReadmes[release].read_bytes().replace(b"\r\n", b"\n").replace(b"\n", lineEnding)
    root = tmp_path / "GMAT install"  # a folder with a space, as users have
    console = _installation(root, platform, release, readme)
    assert console.name == ("GmatConsole.exe" if platform == "windows" else "GmatConsole")
    assert gmat.gmatRelease(root) == release
    monkeypatch.setattr(gmat.subprocess, "run", lambda *a, **k: type("R", (), {"stdout": BANNER})())
    assert gmat.gmatVersion(root, console) == f"GMAT {release}, build Mar 30 2026  10:09:27"


def test_gmat_banner_without_build_date_is_rejected(tmp_path, monkeypatch):
    """Verify a console banner that has no ``Build Date:`` line is rejected even when the release is known."""
    root = tmp_path / "GMAT install"
    console = _installation(root, "windows", "R2026a", README_TEXT)
    noDate = "General Mission Analysis Tool\nConsole Based Version\n"
    monkeypatch.setattr(gmat.subprocess, "run", lambda *a, **k: type("R", (), {"stdout": noDate})())
    with pytest.raises(RuntimeError, match="build date"):
        gmat.gmatVersion(root, console)


def test_gmat_without_release_metadata_is_rejected(tmp_path, monkeypatch):
    """Verify an installation that states no release is rejected instead of given a made-up version."""
    root = tmp_path / "GMAT install"
    console = _installation(root, "windows", "R2026a", README_TEXT)
    (root / "README.txt").unlink()
    monkeypatch.setattr(gmat.subprocess, "run", lambda *a, **k: type("R", (), {"stdout": BANNER})())
    with pytest.raises(RuntimeError, match="release"):
        gmat.gmatVersion(root, console)


def test_gmat_distribution_without_console_is_rejected(tmp_path):
    """Verify a distribution that ships only the GUI (as the Windows R2020a and R2022a zips do) gives a clear error."""
    (tmp_path / "bin").mkdir()
    (tmp_path / "bin" / "GMAT.exe").touch()
    with pytest.raises(RuntimeError, match="GmatConsole"):
        gmat.gmatConsole(tmp_path)


def test_gmat_release_is_never_taken_from_an_example_file_name(tmp_path):
    """Verify names such as ``Ex_R2014a_*.script`` in the distribution cannot be mistaken for the release."""
    root = tmp_path / "GMAT"
    (root / "bin").mkdir(parents=True)
    (root / "bin" / "GmatConsole.exe").touch()
    (root / "samples").mkdir()
    (root / "samples" / "Ex_R2014a_HighFidelitySRP.script").touch()
    assert gmat.gmatRelease(root) is None


def test_gmat_external_data_is_found_with_crlf_startup_file_in_folder_with_space(tmp_path):
    """Verify the data files of a startup file with Windows line endings and ``NAME/`` variables resolve here."""
    root = tmp_path / "GMAT install"
    (root / "bin").mkdir(parents=True)
    startup = ["DATA_PATH = ../data/", "PLANETARY_EPHEM_DE_PATH = DATA_PATH/planetary_ephem/de/",
               "PLANETARY_COEFF_PATH = DATA_PATH/planetary_coeff/", "TIME_PATH = DATA_PATH/time/",
               "DE405_FILE = PLANETARY_EPHEM_DE_PATH/leDE1941.405", "EOP_FILE = PLANETARY_COEFF_PATH/eopc04",
               "LEAP_SECS_FILE = TIME_PATH/tai-utc.dat", "# comment"]
    (root / "bin" / "gmat_startup_file.txt").write_bytes("\r\n".join(startup).encode())
    for rel in ("planetary_ephem/de/leDE1941.405", "planetary_coeff/eopc04", "time/tai-utc.dat"):
        (root / "data" / rel).parent.mkdir(parents=True, exist_ok=True)
        (root / "data" / rel).write_text("x")
    data = gmat.gmatExternalData(root, common.loadSpec())
    assert {"de405_file", "eop_file", "leap_secs_file"} <= set(data)
    assert data["de405_file"]["id"] == "leDE1941.405"


def test_gmat_script_paths_of_this_machine_have_no_backslashes_and_resolve(tmp_path):
    """Verify the temporary file paths written into a GMAT script point to the same files on this operating system."""
    folder = tmp_path / "temp folder"
    folder.mkdir()
    cof, report = folder / "case.cof", folder / "case.txt"
    cof.write_text("x")
    spec = common.loadSpec()
    case = next(c for c in (spec["cases"].values() if isinstance(spec["cases"], dict) else spec["cases"])
                if c["gravity"]["degree"] > 0)
    script = gmat.gmatScript(spec, case, cof, report, cof)
    assert "\\" not in script
    written = [line.split("'")[1] for line in script.splitlines() if "PotentialFile" in line]
    assert written and all(Path(p).resolve() == cof.resolve() for p in written)


class _FakeUrl:
    """Stand-in for ``java.net.URL`` with the escaped path of ``getPath()`` and the URI of ``toURI()``."""

    def __init__(self, uri):
        self.uri = uri

    def getPath(self):
        return self.uri.split("file:", 1)[1]

    def toURI(self):
        return self.uri


def _fakeFile(uri):
    """Stand-in for ``java.io.File(URI)``: decode the URI into the native path, as Java does."""
    from urllib.parse import unquote
    path = unquote(uri.split("file:", 1)[1])
    if path[:3] in ("/C:", "/D:"):  # Windows: drop the leading slash and use backslashes
        path = path[1:].replace("/", "\\")
    return type("File", (), {"getPath": lambda self: path})()


@pytest.mark.parametrize("uri, expected", [
    ("file:/home/me/orekit%20env/orekit-13.jar", "/home/me/orekit env/orekit-13.jar"),
    ("file:/home/me/orekit/orekit-13.jar", "/home/me/orekit/orekit-13.jar"),
    ("file:/C:/Program%20Files/orekit/orekit-13.jar", "C:\\Program Files\\orekit\\orekit-13.jar"),
])
def test_orekit_jar_path_is_unescaped(uri, expected):
    """Verify the jar location becomes a native path string, without URL escaping, for POSIX and Windows paths."""
    import generate_orekit_reference as orekit

    # Path() renders POSIX-style locations with the separators of the running platform
    expectedPath = expected if expected[1:2] == ":" else str(Path(expected))
    assert str(orekit.jarPath(_FakeUrl(uri), _fakeFile)) == expectedPath


@pytest.mark.ciSkip  # deselected in the CI runs: needs orekit_jpype and Java, which the runners do not install
def test_orekit_jar_path_of_this_machine_is_a_native_unescaped_path(tmp_path):
    """Verify the jar of the installed Orekit is located, and hashed, from a folder with a space on this machine."""
    import shutil

    orekit_jpype = pytest.importorskip("orekit_jpype")
    import jpype
    import generate_orekit_reference as orekit

    try:
        if not jpype.isJVMStarted():
            orekit_jpype.initVM()
    except Exception as error:  # no Java runtime
        pytest.skip(f"Java VM unavailable: {error}")
    jars = [j for j in Path(orekit_jpype.__file__).parent.rglob("orekit-*.jar")
            if not j.stem.endswith(("javadoc", "sources"))]
    if not jars:
        pytest.skip("the installed orekit_jpype carries no orekit jar")
    from java.io import File
    from java.net import URLClassLoader

    folder = tmp_path / "orekit env"
    folder.mkdir()
    jar = Path(shutil.copy(jars[0], folder))
    loader = URLClassLoader([File(str(jar)).toURI().toURL()], None)  # no parent: the class comes from this copy
    location = loader.loadClass("org.orekit.frames.FramesFactory").getProtectionDomain().getCodeSource().getLocation()
    path = orekit.jarPath(location, File)
    assert "%20" not in str(path)
    assert path.resolve() == jar.resolve()
    assert len(common.fileSha256(path)) == 64
    assert path.name.startswith("orekit-")  # the release that is recorded in the manifest


@pytest.mark.parametrize("version, accepted", [("12.2.1.2", False), ("12.0.2.0", False), ("13.0.1.0", True),
                                               ("13.1.8.0", True), ("14.0.0.0", True)])
def test_orekit_older_than_the_supported_major_release_is_rejected(version, accepted):
    """Verify the Orekit generator stops with a clear error before it uses an API that older releases lack."""
    import generate_orekit_reference as orekit

    if accepted:
        orekit.requireOrekit(version)
    else:
        with pytest.raises(RuntimeError, match="Orekit 13 or newer"):
            orekit.requireOrekit(version)
