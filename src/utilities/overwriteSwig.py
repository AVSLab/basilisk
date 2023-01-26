#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
import os
import sys

from shutil import move
from tempfile import mkstemp

def overwriteSwigImports(buildDir):
    initFiles = findInitFiles(buildDir)
    for initFile in initFiles:
        replaceImport(initFile)

def findInitFiles(buildDir):
    initFiles = []
    for root, dirs, files in os.walk(buildDir):
        for file in files:
            if file == '__init__.py':
                initFiles.append(os.path.join(root,file))
    return initFiles

def replaceImport(initFile):
    fh, path = mkstemp()
    with os.fdopen(fh,'w+') as newFile:
        with open(initFile) as oldFile:
            for line in oldFile:
                if 'import' in line:
                    importMod = line.split(' ')[1]
                    importMod = importMod.strip('.')
                    if importMod != ".":
                        newFile.write('from .{} import * \n'.format(importMod))
                else:
                    newFile.write(line)
    os.remove(initFile)
    move(path, initFile)

def overwriteSwigCode(buildDir):
    editFiles = findFiles(buildDir)
    for editFile in editFiles:
        replaceCode(editFile)

def findFiles(buildDir):
    editFiles = []
    for root, dirs, files in os.walk(buildDir):
        for file in files:
            if file == 'thrusterDynamicEffector.py' or file == 'reactionWheelStateEffector.py' or file == 'vscmgStateEffector.py':
                editFiles.append(os.path.join(root,file))
    return editFiles

def replaceCode(editFile):
    fh, path = mkstemp()
    with os.fdopen(fh,'w+') as newFile:
        with open(editFile) as oldFile:
            for line in oldFile:
                if 'if(hasattr(self, name) or name == \'this\')' in line:
                    newFile.write(line.replace('name == \'this\'', 'name == \'this\' or name == \'__swig_container\''))
                else:
                    newFile.write(line)
    os.remove(editFile)
    move(path, editFile)

if __name__ == '__main__':
    #Not currently needed. May change import scheme which would require these lines
    #if six.PY3:
    #    overwriteSwigImports(sys.argv[1])
    if int(sys.argv[2][0]) >= 4:
        overwriteSwigCode(sys.argv[1])
