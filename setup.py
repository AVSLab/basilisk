''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''

#------------------------------------------------------------------------------
# XXX: Check that the installation is being performed using a modern Python
# installation tool ("PEP-517 frontend"), with this `setuptools` backend.
# See: https://blog.ganssle.io/articles/2021/10/setup-py-deprecated.html
import os
if os.getenv("PEP517_BUILD_BACKEND") != "setuptools.build_meta":
    raise SystemError("Please install Basilisk using a PEP-517 frontend! e.g. `pip install .` or `python -m build`")
#------------------------------------------------------------------------------


#------------------------------------------------------------------------------
# XXX: Check that `setuptools` is the correct version. This check is necessary
# because older versions of pip<22.3 can install site-package versions instead
# of the requested package versions. But we can't check the `pip` version
# because it runs the PEP-517 frontend, so it may not be available here at all.
# See https://github.com/pypa/pip/issues/6264
from importlib.metadata import version  # Supported on Python 3.8+
setuptools_version = version('setuptools')
if int(setuptools_version.split(".")[0]) < 64:
    raise RuntimeError(f"setuptools>=64 is required to install Basilisk, but found setuptools=={setuptools_version}. " \
                       f"This can happen on some old versions of pip (upgrade with `pip install --upgrade \"pip>=22.3\"`).")
#------------------------------------------------------------------------------


#------------------------------------------------------------------------------
# Allow user to pass arguments to Conan through environment variables.
# TODO: Should allow these to be passed in as arguments, e.g. pip's
# `--config-settings`. However, this is not implemented by setuptools yet.
# See https://github.com/pypa/setuptools/issues/3896
import shlex
EXTERNAL_MODULES = os.getenv("EXTERNAL_MODULES")
USER_CONAN_ARGS  = os.getenv("CONAN_ARGS")
CONAN_ARGS       = [
    *(["-o", f"pathToExternalModules={EXTERNAL_MODULES}"] if EXTERNAL_MODULES else []),
    *(shlex.split(USER_CONAN_ARGS) if USER_CONAN_ARGS else []),
]
#------------------------------------------------------------------------------

from setuptools import setup, Command, Extension, find_packages
from setuptools.command.build import build, SubCommand

from conan.api.conan_api import ConanAPI
from conan.cli.cli import Cli as ConanCLI
from conan.errors import ConanException

from typing import List
from pathlib import Path

THIS_DIR = Path(__file__).parent.resolve()

# Override build commands to add the ConanExtension builder.
# (These must run first, before `build_py`.)
build.sub_commands.insert(0, ('build_conan', build.has_ext_modules))

class ConanExtension(Extension):
    def __init__(self, name: str, src: Path, dest: Path, build_type: str = "Release", args: List=[]):
        self.name = name
        self.src = Path(src).resolve()
        self.dest = Path(dest).resolve()
        self.conanfile = self.src/"conanfile.py"
        self.build_type = build_type
        self.args = args

        assert self.conanfile.is_file(), f"Expected to find conanfile.py file at {self.conanfile}"

        # Initialise as an Extension so that setuptools builds a "platform
        # wheel" (not a "pure Python wheel"). We set optional=True to allow
        # `build_ext` to fail to build this empty extension.
        super().__init__(name, sources=[], optional=True)

class BuildConanExtCommand(Command, SubCommand):
    """
    Custom builder for Conan extension models.
    """
    description = "\"build\" Conan extension modules"

    def initialize_options(self) -> None:
        self.conan_extensions = []

    def finalize_options(self) -> None:
        self.conan_extensions = [ext for ext in self.distribution.ext_modules if isinstance(ext, ConanExtension)]

    def get_source_files(self) -> List[str]:
        return [ext.conanfile.relative_to(THIS_DIR).as_posix() for ext in self.conan_extensions]

    def run(self) -> None:
        conan_api = ConanAPI()
        conan = ConanCLI(conan_api)

        for ext in self.conan_extensions:
            try:
                # Create a default conan profile.
                # TODO: Do we want to --force? Probably not...
                conan.run(["profile", "detect"])
            except ConanException:
                pass  # The user already has a conan profile (or another issue).

            # Build the Conanfile with the appropriate settings.
            conan.run([
                "build", ext.src.as_posix(),
                "--build", "missing",  # Build any packages that can't be downloaded.
                "-s", f"build_type={ext.build_type}",
                "-o", f"buildFolder={ext.dest.as_posix()}",
                "-o", f"clean=True", # XXX: Clean build by default, to avoid various issues!
                *ext.args])

            # Find packages built by this extension and add them to Distribution.
            for pkg in find_packages(ext.dest):
                pkg_dir = Path(ext.dest, *pkg.split("."))
                self.distribution.packages.append(pkg)
                self.distribution.package_dir[pkg] = os.path.relpath(pkg_dir, start=THIS_DIR)

            # Refresh `build_py` to ensure it can find and copy the packages.
            # NOTE: Leave this extension in self.distribution.ext_modules to
            # ensure that setuptools builds this as a "platform Wheel".
            self.reinitialize_command("build_py")

setup(
    packages = [],  # XXX: Leave empty! (Automatically set by ConanExtension)

    include_package_data = True,
    package_data = {
        "": ["*.so", "*.dll", "*.lib", "*.pyd"],  # Include all built objects.
        "Basilisk": ["supportData/**/*"]          # Include all support data.
    },

    ext_modules=[
        # Build the Conanfile as an Extension, which ensures that setuptools builds
        # this as a "platform wheel" instead of a "pure Python wheel".
        ConanExtension(
            name="Basilisk",
            src=THIS_DIR,
            dest="dist3",
            build_type="Release",
            args=CONAN_ARGS,
        )
    ],

    cmdclass={
        'build_conan': BuildConanExtCommand,
    },
)