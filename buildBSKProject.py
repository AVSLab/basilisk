# ISC License
#
# Copyright (c) 2020, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import argparse
import errno
import os
import platform
import shutil

parser = argparse.ArgumentParser(description="Build the Basilisk framework.")

# define the optional arguments
parser.add_argument("--python2", help="build for Python 2", action="store_true")
parser.add_argument("--buildTarget", help="build target")
parser.add_argument("--buildType", help="build type", default="Release", choices=["Release", "Debug"])
parser.add_argument("--buildOpNav", help="build modules for OpNav behavior", default=False,
                    type=lambda x: (str(x).lower() == 'true'))
parser.add_argument("--buildVizInterface", help="build vizInterface module", default=True,
                    type=lambda x: (str(x).lower() == 'true'))
parser.add_argument("--clean", help="make a clean distribution folder", action="store_true")
args = parser.parse_args()

# set the build destination folder
if args.python2:
    buildFolderName = 'dist'
    print("Building for Python 2 (depreciated)")
else:
    buildFolderName = 'dist3'
buildDestination = os.getcwd() + '/' + buildFolderName

# clean out the distribution folder if needed
if os.path.exists(buildDestination) and args.clean:
    # removing the file using the os.remove() method
    shutil.rmtree(buildDestination)

# make the destination folder if it does not exist
if not os.path.exists(buildDestination):
    try:
        os.makedirs(buildDestination)
        print("Created the build folder: " + buildFolderName)
    except OSError as exc:  # Guard against race condition
        print("Could not make build folder.")
        if exc.errno != errno.EEXIST:
            raise
else:
    print("Using the build folder: " + buildFolderName)
os.chdir(buildDestination)

# determine the build target
if not args.buildTarget:    # if this isn't defined then set the build target from the OS
    osPlatform = platform.system()
    if osPlatform == 'Darwin':
        print('Found macOS Operating system and building for: Xcode')
        buildTarget = 'Xcode'
    elif osPlatform == 'Linux':
        print('Found Linux operating system and building: Unix Makefiles.')
        buildTarget = '"Unix Makefiles"'
    else:
        print('Could not determine build-target automatically')
        exit(1)
else:
    buildTarget = args.buildTarget
    print('Build target set to: ' + buildTarget)

# run conan to check all dependency packages are installed
conanCmdString = 'conan install .. -s build_type=Release --build=missing -if cmake'
conanCmdString += ' -o opnav_packages=' + str(args.buildOpNav)
conanCmdString += ' -o vizInterface_packages=' + str(args.buildVizInterface)
print("Running conan:")
os.system(conanCmdString)

# run cmake
cmakeCmdString = 'cmake ../src -G ' + buildTarget
cmakeCmdString += ' -DBUILD_OPNAV=' + str(args.buildOpNav)
cmakeCmdString += ' -DBUILD_VIZINTERFACE=' + str(args.buildVizInterface)
if args.python2:
    cmakeCmdString += ' -DUSE_PYTHON3=OFF'
cmakeCmdString += ' -DCMAKE_BUILD_TYPE=' + args.buildType
print("Running cmake:")
print(cmakeCmdString)
os.system(cmakeCmdString)