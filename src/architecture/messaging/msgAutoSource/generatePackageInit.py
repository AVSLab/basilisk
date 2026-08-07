import os
import sys

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(path + '/../../../../../Basilisk/src/architecture/messaging/msgAutoSource')


def generatePackageInit(moduleOutputPath, headerInputPaths):
    """Generate deterministic imports for the Python messaging package.

    :param moduleOutputPath: directory where the package initializer is written
    :param headerInputPaths: directories containing message payload headers
    """
    os.makedirs(moduleOutputPath, exist_ok=True)
    outputPath = os.path.join(moduleOutputPath, '__init__.py')
    with open(outputPath, 'w', encoding='utf-8') as mainImportFile:
        mainImportFile.write(
            'from Basilisk.architecture.messaging.messagingSupport import *\n'
        )
        for headerInputPath in headerInputPaths:
            for fileName in sorted(os.listdir(headerInputPath)):
                if fileName.endswith(('.h', '.hpp')):
                    className = os.path.splitext(fileName)[0]
                    mainImportFile.write(
                        'from Basilisk.architecture.messaging.'
                        + className
                        + ' import *\n'
                    )
        mainImportFile.write('from Basilisk.architecture.messagingBase import *\n')


if __name__ == "__main__":
    moduleOutputPath = sys.argv[1]
    generatePackageInit(moduleOutputPath, sys.argv[2:])
    setOldPath = moduleOutputPath.split('messaging')[0] + '/cMsgCInterfacePy'

    # XXX: Disabled: don't make a symbolic link here, because when we build a
    # Python wheel, the contents of the folder get copied, effectively doubling
    # the size. Instead, see the new messaging/cMsgCInterface directory.
    # os.symlink(moduleOutputPath, setOldPath)
