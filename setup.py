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

#-------------------------------------------------------------------------------
# XXX: Check that `setuptools` is the correct version. This check is necessary
# because older versions of pip<22.3 can install site-package versions instead
# of the requested package versions. But we can't check the `pip` version
# because it runs the PEP-517 frontend, so it may not be available here at all.
# So instead, we check it indirectly by ensuring that setuptools is an
# acceptable version. See https://github.com/pypa/pip/issues/6264
from importlib.metadata import version  # Supported on Python 3.8+
setuptools_version = version('setuptools')
if int(setuptools_version.split(".")[0]) < 64:
    raise RuntimeError(f"setuptools>=64 is required to install Basilisk, but found setuptools=={setuptools_version}. " \
                       f"This can happen on old versions of pip. Please upgrade with `pip install --upgrade \"pip>=22.3\"`.")
#-------------------------------------------------------------------------------


#-------------------------------------------------------------------------------
# Allow user to pass arguments to Conan through environment variables.
# TODO: Should allow these to be passed in as arguments, e.g. pip's
# `--config-settings`. However, this is not implemented by setuptools yet.
# See https://github.com/pypa/setuptools/issues/3896
import shlex
import os
USER_CONAN_ARGS = shlex.split(os.getenv("CONAN_ARGS") or "")
#-------------------------------------------------------------------------------


import sys
from dataclasses import dataclass
from setuptools import setup, Command, Extension, find_packages
from setuptools.command.build import build, SubCommand
from subprocess import run
from pathlib import Path

HERE = Path(__file__).parent.resolve()


@dataclass
class ConanExtension(Extension):
    name: str
    src: Path
    build_dir: str
    args: list[str]

    def __post_init__(self):
        self.conanfile = Path(self.src)/"conanfile.py"
        assert self.conanfile.is_file(), f"Expected to find conanfile.py file at {self.conanfile}"


class BuildConanExtCommand(Command, SubCommand):
    def initialize_options(self) -> None:
        self.conan_extensions = []

    def finalize_options(self) -> None:
        # NOTE: Leave the extensions in self.distribution.ext_modules to
        # ensure that setuptools builds this as a "platform Wheel".
        self.conan_extensions = [ext for ext in self.distribution.ext_modules if isinstance(ext, ConanExtension)]

        # Set limited ABI compatibility by default, targeting the minimum required Python version.
        # See https://docs.python.org/3/c-api/stable.html
        # NOTE: Swig 4.2.1 or higher is required, see https://github.com/swig/swig/pull/2727
        min_version = next(self.distribution.python_requires.filter([f"3.{i}" for i in range(2, 100)])).replace(".", "")
        bdist_wheel = self.reinitialize_command("bdist_wheel", py_limited_api=f"cp{min_version}")
        bdist_wheel.ensure_finalized()
        for ext in self.conan_extensions:
            ext.args += ["--pyLimitedAPI", f"0x{min_version[0]:>02}{min_version[1]:>02}00f0"]

    def get_source_files(self) -> list[str]:
        # NOTE: This is necessary for building sdists, and is populated
        # automatically by setuptools-scm in the project build-requires.
        return []

    def run(self) -> None:
        for ext in self.conan_extensions:
            if self.editable_mode:
                # TODO: Add support for installing in editable mode.
                built = any(Path(ext.build_dir).glob("Basilisk*"))
                if not built:
                    run([sys.executable, ext.conanfile] + ext.args, check=True)
            else:
                # Call the underlying Conanfile with the desired arguments.
                run([sys.executable, ext.conanfile] + ext.args, check=True)

            # Find packages built by this extension and add them to the Distribution.
            for pkg in find_packages(ext.build_dir):
                pkg_dir = Path(ext.build_dir, *pkg.split("."))
                self.distribution.packages.append(pkg)
                self.distribution.package_dir[pkg] = os.path.relpath(pkg_dir, start=HERE)

                pd = self.distribution.package_data.setdefault(pkg, [])
                pd += ["*.dll", "**/*.dll", "*.pyd", "**/*.pyd"]

        if self.editable_mode and len(self.distribution.packages) == 0:
            raise Exception("Tried to install in editable mode, but packages have not been prepared yet! " \
                            "Please install via `python conanfile.py` instead!")

        # Refresh `build_py` to ensure it can find the newly generated packages.
        build_py = self.reinitialize_command("build_py")
        build_py.ensure_finalized()


# XXX: Forcibly override build to run ConanExtension builder before build_py.
build.sub_commands = [
    ('build_ext', build.has_ext_modules),
    ('build_py', build.has_pure_modules)
]


setup(
    ext_modules=[
        # XXX: Build as an "extension" to force "has_ext_modules" to be True,
        # and build this as a "platform wheel" instead of a "pure Python wheel".
        ConanExtension(
            name="Basilisk",
            build_dir="dist3",  # XXX: Hard-coded in conanfile, leave this as is!
            src=HERE,
            args=[
                # (defaults)
                "--buildType", "Release",
                "--buildProject", "True",
                "--clean",
                # (user arguments)
                *USER_CONAN_ARGS,
                # (overrides)
                "--managePipEnvironment", "False"  # Force conanfile to leave pip alone.
            ]
        )
    ],

    url="https://avslab.github.io/basilisk/",  # Ensure this field is populated

    # XXX: Override build_ext with ConanExtension builder.
    cmdclass={'build_ext': BuildConanExtCommand},
    zip_safe=False,
    include_package_data=True,
)
