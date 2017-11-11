import os
import sys

from setuptools import Command, setup
from setuptools.command.test import test as TestCommand

# Just a function to run a command
verbose = False


def runCommand(cmd):
    if not verbose:
        cmd += "> /dev/null"
    print "Running command:", cmd
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
    description = "Custom clean command that removes dist/build and artifacts"
    user_options = []

    def initialize_options(self):
        self.cwd = None

    def finalize_options(self):
        self.cwd = os.getcwd()

    def run(self):
        to_delete = [
            "dist/",
            "docs/built",
            "docs/Images/Scenarios"
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
        print "Making distribution directory"
        runCommand("mkdir dist/")
        print "Executing CMake build into dist/ directory"
        # if we switch to using mostly setup.py for the build, install will not be done by CMake
        print "This also will install Basilisk locally..."
        runCommand("cd dist && cmake -G Xcode ../src/")


class XCodeBuildCommand(Command):
    description = "Custom build command that runs XCode"
    user_options = []

    def initialize_options(self):
        self.cwd = None

    def finalize_options(self):
        self.cwd = os.getcwd()

    def run(self):
        assert os.getcwd() == self.cwd, 'Must be in package root: %s' % self.cwd
        print "Executing XCode build into dist/ directory"
        runCommand("xcodebuild -project dist/AVS\ basilisk.xcodeproj -target ALL_BUILD")


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
        print "Executing linter"
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
        print "Building documentation"
        runCommand("doxygen docs/DoxyData")


setup(
    name='Basilisk',
    version='0.1.6',
    description="Astrodynamic Simulation Library",
    packages=['Basilisk', ],
    license=open('./LICENSE').read(),
    long_description=open('./README.md').read(),
    author_email='basilisk-info@colorado.edu',
    url='http://hanspeterschaub.info/bskMain.html',
    package_dir={'': 'dist'},
    install_requires=[
        'matplotlib',
        'numpy'
    ],
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
