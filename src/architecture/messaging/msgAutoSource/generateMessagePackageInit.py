import os
import pathlib
import sys

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(path + '/../../../../../Basilisk/src/architecture/messaging/msgAutoSource')

if __name__ == "__main__":
    """
    In the message package directory create an __init__.py file
    containing all the message packages. If the file exists, overwrite it.
    """
    modulePath = sys.argv[1]
    isExist = os.path.exists(modulePath)
    os.makedirs(modulePath, exist_ok=True)
    mainImportFid = open(modulePath + '/__init__.py', 'w')
    for i in range(2, len(sys.argv)):
        headerInputPath = sys.argv[i]
        for fileName in os.listdir(headerInputPath):
            if fileName.endswith(".h") or fileName.endswith(".hpp"):
                className = os.path.splitext(fileName)[0]
                msgName = className.split('Payload')[0]
                mainImportFid.write('from Basilisk.architecture.messaging.' + className + ' import *\n')
    mainImportFid.close()
    oldModulePath = modulePath.split('messaging')[0] + '/cMsgCInterfacePy'
    pathlib.Path(oldModulePath).unlink(missing_ok=True)
    os.symlink(modulePath, oldModulePath)
