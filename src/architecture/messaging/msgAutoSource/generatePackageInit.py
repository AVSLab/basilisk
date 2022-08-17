import parse
import os,errno
import shutil
import sys

path = os.path.dirname(os.path.abspath(__file__))
# This part definitely needs work.  Need to detect Basilisk somehow.
sys.path.append(path + '/../../../../../Basilisk/src/architecture/messaging/msgAutoSource')

if __name__ == "__main__":
    moduleOutputPath = sys.argv[1]
    mainImportFid = open(moduleOutputPath + '/__init__.py', 'w')
    #mainImportFid.write('from Basilisk.architecture.MessagingToplevel import *\n')
    for i in range(2, len(sys.argv)):
        headerInputPath = sys.argv[i]
        for filePre in os.listdir(headerInputPath):
            if(filePre.endswith(".h") or filePre.endswith(".hpp")):
                className = os.path.splitext(filePre)[0]
                msgName = className.split('Payload')[0]
                mainImportFid.write('from Basilisk.architecture.messaging.' + className + ' import *\n')
                #mainImportFid.write('from Basilisk.architecture.messaging.' + className + ' import ' + className +' as ' + className + '\n')
                #mainImportFid.write('from Basilisk.architecture.messaging.' + className + ' import ' + msgName +' as ' + msgName + '\n')
                #mainImportFid.write('from Basilisk.architecture.messaging.' + className + ' import ' + msgName + 'Recorder' + ' as ' + msgName + 'Recorder'+ '\n')
                #mainImportFid.write('from Basilisk.architecture.messaging.' + className + ' import ' + className + 'Vector' + ' as ' + msgName + 'Vector' + '\n')
    mainImportFid.close()
