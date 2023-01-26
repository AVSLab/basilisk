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
import os
import sys

from setuptools import Command, setup
from setuptools.command.test import test as TestCommand

# Just a function to run a command
verbose = False


def runCommand(cmd, dir=None):
    if not verbose:
        cmd += "> /dev/null"
    print("Running command:", cmd)

    originalDir = os.getcwd()
    if dir is not None:
        os.chdir(dir)
        os.system(cmd)
        os.chdir(originalDir)
    else:
        os.system(cmd)


class PyTestCommand(TestCommand):
    # Here we define a command to use for testing installed Basilisk
    # Taken from pytest documentation found https://docs.pytest.org/en/latest/goodpractices.html
    description = "Custom test command that runs pytest"
    user_options = [('pytest-args=', 'a', "Arguments to pass to pytest")]

    def initialize_options(self):
        TestCommand.initialize_options(self)
        self.pytest_args = ''

    def run_tests(self):
        import shlex
        # import here, cause outside the eggs aren't loaded
        import pytest
        errno = pytest.main(['src'] + shlex.split(self.pytest_args))
        sys.exit(errno)


class CleanCommand(Command):
    # Custom command to clean up
    description = "Custom clean command that removes dist3/build and artifacts"
    user_options = []

    def initialize_options(self):
        self.cwd = None

    def finalize_options(self):
        self.cwd = os.getcwd()

    def run(self):
        to_delete = [
            "dist3/",
            "docs/build",
            "docs/source/_images/Scenarios"
        ]
        assert os.getcwd() == self.cwd, 'Must be in package root: %s' % self.cwd
        runCommand('rm -rf ' + " ".join(to_delete))


class CMakeBuildCommand(Command):
    # Custom command to build with cmake and xcode
    description = "Custom build command that runs CMake"
    user_options = []

    def initialize_options(self):
        self.cwd = None

    def finalize_options(self):
        self.cwd = os.getcwd()

    def run(self):
        assert os.getcwd() == self.cwd, 'Must be in package root: %s' % self.cwd
        print("Making distribution directory")
        runCommand("mkdir dist3/")
        print("Executing CMake build into dist3/ directory")
        # if we switch to using mostly setup.py for the build, install will not be done by CMake
        print("This also will install Basilisk locally...")
        runCommand("cmake -G Xcode ../src/", "dist3/")


class XCodeBuildCommand(Command):
    description = "Custom build command that runs XCode"
    user_options = []

    def initialize_options(self):
        self.cwd = None

    def finalize_options(self):
        self.cwd = os.getcwd()

    def run(self):
        assert os.getcwd() == self.cwd, 'Must be in package root: %s' % self.cwd
        print("Executing XCode build into dist3/ directory")
        runCommand("xcodebuild -project dist3/basilisk.xcodeproj -target ALL_BUILD")


# Lint command
class LintCommand(Command):
    description = "Custom lint command that displays pep8 violations"
    user_options = []

    def initialize_options(self):
        self.cwd = None

    def finalize_options(self):
        self.cwd = os.getcwd()

    def run(self):
        assert os.getcwd() == self.cwd, 'Must be in package root: %s' % self.cwd
        print("Executing linter")
        runCommand("flake8 src/")


class BuildDocsCommand(Command):
    # Custom command to build with cmake and xcode

    description = "Custom build command to build the documentation with doxygen"
    user_options = []

    def initialize_options(self):
        self.cwd = None

    def finalize_options(self):
        self.cwd = os.getcwd()

    def run(self):
        assert os.getcwd() == self.cwd, 'Must be in package root: %s' % self.cwd
        print("Building documentation")
        runCommand("make html", "docs/source")

package_dir = "dist3"

f = open('docs/source/bskVersion.txt', 'r')
bskVersion = f.read().strip()

setup(
    name='Basilisk',
    version=bskVersion,
    description="Astrodynamics Simulation Library",
    packages=['Basilisk', ],
    license=open('./LICENSE').read(),
    long_description=open('./README.md').read(),
    url='https://hanspeterschaub.info/basilisk/',
    package_dir={'': package_dir},
    # install_requires=[
    #     'matplotlib',
    #     'numpy',
    #     'pandas'
    # ],
    setup_requires=['pytest-runner'],
    tests_require=['pytest', 'flake8'],
    cmdclass={
        'clean': CleanCommand,
        'xcode': XCodeBuildCommand,
        'cmake': CMakeBuildCommand,
        'test': PyTestCommand,
        'docs': BuildDocsCommand,
        'lint': LintCommand
    }
)
