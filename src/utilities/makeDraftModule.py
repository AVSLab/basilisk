# ISC License
#
# Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

"""
This script is used to create a Basilisk module folder given the basic I/O and naming information.

- Modify either ``fillCppInfo()`` or ``fillCInfo()`` to contain the desired information for the new BSK module.
- edit the ``__main__`` routine at the end of the file to call the desired module type with
  ``createCppModule()`` or ``createCModule``.
- run the script from the command line using ``python3 makeDraftModule.py``

"""

import os
import re
import shutil
from datetime import datetime

# assumes this script is in .../basilisk/src/utilities
pathToSrc = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
initialCwd = os.getcwd()

statusColor = '\033[92m'
failColor = '\033[91m'
warningColor = '\033[93m'
endColor = '\033[0m'

class moduleGenerator:
    """
    class to generate draft Basilisk modules
    """
    def __init__(self):
        # the following variables must be set for this module generator to function
        self.modulePathRelSrc = None  # path to the new module folder relative to basilisk/src
        self.moduleName = None  # lower camel case name of the module
        self.briefDescription = None  # brief module description
        self.copyrightHolder = None  # holder of open source copyright
        self.inMsgList = []  # list of input message dictionary list
        self.outMsgList = []  # list of input message dictionary list

        # module behavior flags
        self.cleanBuild = False  # flag if any prior directories should be deleted automatically
        self.verbose = True  # flag if the status messages should be printed

        # private class variables
        self._absPath = None  # absolute path to the folder which will contain the module folder
        self._newModuleLocation = None  # absolute path to the auto-generated Basilisk module folder
        self._licenseText = None  # BSK open-source license statement

    def log(self, statement, **kwargs):
        if self.verbose:
            if 'end' in kwargs:
                endString = kwargs['end']
                print(statement, end=endString)
            else:
                print(statement)

    def checkPathToNewFolderLocation(self):
        """
        Make sure the supplied module destination path is correct
        """
        self.log(statusColor + "Checking Module location:" + endColor, end=" ")
        if os.path.isdir(self._absPath):
            os.chdir(self._absPath)
        else:
            self.log(failColor + "\nERROR: " + endColor + "Incorrect path to the new folder:")
            self.log(self._absPath)
            exit()
        self.log("Done")
        self.log(self._absPath)

    def createNewModuleFolder(self):
        """
        Create the new module folder
        """
        self.log(statusColor + "Creating Module Folder:" + endColor, end=" ")
        if os.path.isdir(self._newModuleLocation):
            self.log("\n" + warningColor + "WARNING: " + endColor + "The new module destination already exists.")
            if not self.cleanBuild:
                ans = input("Do you want to delete this folder and recreate? (y or n): ")
                if ans != "y":
                    self.log(failColor + "Aborting module creation." + endColor)
                    exit()
            self.log("Cleared the old folder.")
            shutil.rmtree(self._newModuleLocation)
        else:
            self.log("Done")
        os.mkdir(self._newModuleLocation)
        os.chdir(self._newModuleLocation)

    def readLicense(self):
        """Read the Basilisk license file"""
        self.log(statusColor + "Importing License:" + endColor, end=" ")
        with open(pathToSrc + "/../LICENSE", 'r') as f:
            self._licenseText = f.read()
            self._licenseText = self._licenseText.replace("2016", str(datetime.now().year))
            self._licenseText = self._licenseText.replace(
                "Autonomous Vehicle Systems Lab, University of Colorado at Boulder",
                self.copyrightHolder)
        self.log("Done")

    def createRstFile(self):
        """Create the Module RST documentation draft."""
        rstFileName = self.moduleName + ".rst"
        self.log(statusColor + "Creating RST Documentation File " + rstFileName + ":" + endColor, end=" ")
        rstFile = 'Executive Summary\n'
        rstFile += '-----------------\n'
        rstFile += self.briefDescription + '\n'
        rstFile += '\n'
        rstFile += 'Message Connection Descriptions\n'
        rstFile += '-------------------------------\n'
        rstFile += 'The following table lists all the module input and output messages.  \n'
        rstFile += 'The module msg connection is set by the user from python.  \n'
        rstFile += 'The msg type contains a link to the message structure definition, while the description \n'
        rstFile += 'provides information on what this message is used for.\n'
        rstFile += '\n'
        rstFile += '.. list-table:: Module I/O Messages\n'
        rstFile += '    :widths: 25 25 50\n'
        rstFile += '    :header-rows: 1\n'
        rstFile += '\n'
        rstFile += '    * - Msg Variable Name\n'
        rstFile += '      - Msg Type\n'
        rstFile += '      - Description\n'
        for msg in self.inMsgList + self.outMsgList:
            rstFile += '    * - ' + msg['var'] + '\n'
            rstFile += '      - :ref:`' + msg['type'] + 'Payload`\n'
            rstFile += '      - ' + msg['desc'] + '\n'
        rstFile += '\n'

        with open(rstFileName, 'w') as w:
            w.write(rstFile)
        self.log("Done")

    def createTestFile(self, type):
        """
            Create a functioning python unit test file that loads the new module, creates and connect blank
            input messages, and sets up recorder modules for each output message.
        """
        os.mkdir('_UnitTest')
        os.chdir('_UnitTest')
        testFileName = "test_" + self.moduleName + ".py"
        self.log(statusColor + "Creating Python Init Test File " + testFileName + ":" + endColor, end=" ")
        testFile = ""
        for line in self._licenseText.split('\n'):
            testFile += '# ' + line + '\n'
        testFile += '\n'
        testFile += 'import pytest\n'
        testFile += '\n'
        testFile += 'from Basilisk.utilities import SimulationBaseClass\n'
        testFile += 'from Basilisk.utilities import unitTestSupport\n'
        testFile += 'from Basilisk.architecture import messaging\n'
        testFile += 'from Basilisk.utilities import macros\n'
        testFile += 'from Basilisk.' + os.path.split(self.modulePathRelSrc)[0] + ' import ' + self.moduleName + '\n'
        testFile += '\n'
        testFile += '@pytest.mark.parametrize("accuracy", [1e-12])\n'
        testFile += '@pytest.mark.parametrize("param1, param2", [\n'
        testFile += '     (1, 1)\n'
        testFile += '    ,(1, 3)\n'
        testFile += '])\n'
        testFile += '\n'
        testFile += 'def test_' + self.moduleName + '(show_plots, param1, param2, accuracy):\n'
        testFile += '    r"""\n'
        testFile += '    **Validation Test Description**\n'
        testFile += '\n'
        testFile += '    Compose a general description of what is being tested in this unit test script.\n'
        testFile += '\n'
        testFile += '    **Test Parameters**\n'
        testFile += '\n'
        testFile += '    Discuss the test parameters used.\n'
        testFile += '\n'
        testFile += '    Args:\n'
        testFile += '        param1 (int): Dummy test parameter for this parameterized unit test\n'
        testFile += '        param2 (int): Dummy test parameter for this parameterized unit test\n'
        testFile += '        accuracy (float): absolute accuracy value used in the validation tests\n'
        testFile += '\n'
        testFile += '    **Description of Variables Being Tested**\n'
        testFile += '\n'
        testFile += '    Here discuss what variables and states are being checked. \n'
        testFile += '    """\n'
        testFile += '    [testResults, testMessage] = ' + self.moduleName + 'TestFunction(show_plots, param1, param2, accuracy)\n'
        testFile += '    assert testResults < 1, testMessage\n'
        testFile += '\n'
        testFile += '\n'
        testFile += 'def ' + self.moduleName + 'TestFunction(show_plots, param1, param2, accuracy):\n'
        testFile += '    """Test method"""\n'
        testFile += '    testFailCount = 0\n'
        testFile += '    testMessages = []\n'
        testFile += '    unitTaskName = "unitTask"\n'
        testFile += '    unitProcessName = "TestProcess"\n'
        testFile += '\n'
        testFile += '    unitTestSim = SimulationBaseClass.SimBaseClass()\n'
        testFile += '    testProcessRate = macros.sec2nano(0.5)\n'
        testFile += '    testProc = unitTestSim.CreateNewProcess(unitProcessName)\n'
        testFile += '    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))\n'
        testFile += '\n'
        testFile += '    # setup module to be tested\n'
        if type == "C++":
            testFile += '    module = ' + self.moduleName + '.' + self._className + '()\n'
        elif type == "C":
            testFile += '    module = ' + self.moduleName + '.' + self.moduleName + '()\n'
        else:
            self.log(failColor + "ERROR: " + endColor + "Wrong module type provided to test file method.")
            exit(0)
        testFile += '    module.ModelTag = "' + self.moduleName + 'Tag"\n'
        testFile += '    unitTestSim.AddModelToTask(unitTaskName, module)\n'
        testFile += '\n'
        testFile += '    # Configure blank module input messages\n'
        for msg in self.inMsgList:
            testFile += '    ' + msg['var'] + 'Data = messaging.' + msg['type'] + 'Payload()\n'
            testFile += '    ' + msg['var'] + ' = messaging.' + msg['type'] + '().write(' + msg['var'] + 'Data)' + '\n'
            testFile += '\n'
        testFile += '    # subscribe input messages to module\n'
        for msg in self.inMsgList:
            testFile += '    module.' + msg['var'] + '.subscribeTo(' + msg['var'] + ')\n'
        testFile += '\n'
        testFile += '    # setup output message recorder objects\n'
        for msg in self.outMsgList:
            testFile += '    ' + msg['var'] + 'Rec = module.' + msg['var'] + '.recorder()\n'
            testFile += '    unitTestSim.AddModelToTask(unitTaskName, ' + msg['var'] + 'Rec)\n'
        testFile += '\n'
        testFile += '    unitTestSim.InitializeSimulation()\n'
        testFile += '    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))\n'
        testFile += '    unitTestSim.ExecuteSimulation()\n'
        testFile += '\n'
        testFile += '    # pull module data and make sure it is correct\n'
        testFile += '\n'
        testFile += '    if testFailCount == 0:\n'
        testFile += '        print("PASSED: " + module.ModelTag)\n'
        testFile += '    else:\n'
        testFile += '        print(testMessages)\n'
        testFile += '\n'
        testFile += '    return [testFailCount, "".join(testMessages)]\n'
        testFile += '\n'
        testFile += '\n'
        testFile += 'if __name__ == "__main__":\n'
        testFile += '    test_' + self.moduleName + '(False, 1, 1, 1e-12)\n'
        testFile += '\n'
        testFile += '\n'

        with open(testFileName, 'w') as w:
            w.write(testFile)
        self.log("Done")

    def createCppModule(self):
        """
        Create a C++ Basilisk module
        """
        modulePath = self.modulePathRelSrc
        name = self.moduleName
        briefDescription = self.briefDescription
        inMsgList = self.inMsgList
        outMsgList = self.outMsgList

        self.log(statusColor + '\nCreating C++ Module: ' + endColor + name)
        self._className = re.sub('([a-zA-Z])', lambda x: x.groups()[0].upper(), name, 1)

        # read in the license information
        self.readLicense()
        licenseC = "/*" + self._licenseText + "*/\n\n"

        # make sure the path, specified relative to basilisk/src, to the new module location is correct
        self._absPath = os.path.join(pathToSrc, modulePath)
        self.checkPathToNewFolderLocation()

        # create new Module folder
        self._newModuleLocation = os.path.join(self._absPath, name)
        self.createNewModuleFolder()

        #
        # make module header file
        #
        headerFileName = name + ".h"
        self.log(statusColor + "Creating Header File " + headerFileName + ":" + endColor, end=" ")
        headerFile = licenseC
        headerFile += '\n'
        headerFile += '#ifndef ' + name.upper() + '_H\n'
        headerFile += '#define ' + name.upper() + '_H\n'
        headerFile += '\n'
        headerFile += '#include "architecture/_GeneralModuleFiles/sys_model.h"\n'
        # loop over message definition includes
        includedMsgs = []
        for msg in inMsgList + outMsgList:
            # ensure we don't include message definition files multiple times
            if msg['type'] not in includedMsgs:
                if msg['wrap'] == 'C':
                    headerFile += '#include "architecture/msgPayloadDefC/' + msg['type'] + 'Payload.h"\n'
                if msg['wrap'] == 'C++':
                    headerFile += '#include "architecture/msgPayloadDefCpp/' + msg['type'] + 'Payload.h"\n'
                includedMsgs.append(msg['type'])
        headerFile += '#include "architecture/utilities/bskLogging.h"\n'
        headerFile += '#include "architecture/messaging/messaging.h"\n'
        headerFile += '\n'
        headerFile += '/*! @brief ' + briefDescription + '\n */\n'
        headerFile += 'class ' + self._className + ': public SysModel {\n'
        headerFile += 'public:\n'
        headerFile += '    ' + self._className + '();\n'
        headerFile += '    ~' + self._className + '();\n'
        headerFile += '\n'
        headerFile += '    void Reset(uint64_t CurrentSimNanos);\n'
        headerFile += '    void UpdateState(uint64_t CurrentSimNanos);\n'
        headerFile += '\n'
        headerFile += 'public:\n'
        for msg in inMsgList:
            headerFile += '    ReadFunctor<' + msg['type'] + 'Payload> ' + msg['var'] \
                          + ';  //!< ' + msg['desc'] + '\n'
        headerFile += '\n'
        for msg in outMsgList:
            headerFile += '    Message<' + msg['type'] + 'Payload> ' + msg['var'] \
                          + ';  //!< ' + msg['desc'] + '\n'
        headerFile += '\n'
        headerFile += '    BSKLogger bskLogger;              //!< -- BSK Logging\n'
        headerFile += '\n'
        headerFile += '};\n'
        headerFile += '\n'
        headerFile += "\n#endif\n"

        with open(headerFileName, 'w') as w:
            w.write(headerFile)
        self.log("Done")

        #
        # make module definition file
        #
        defFileName = name + ".cpp"
        self.log(statusColor + "Creating Definition File " + defFileName + ":" + endColor, end=" ")
        defFile = licenseC
        defFile += '\n'
        defFile += '#include "' + modulePath + '/' + name + '/' + name + '.h"\n'
        defFile += '#include <iostream>\n'
        defFile += '#include <cstring>\n'
        defFile += '\n'
        defFile += '/*! This is the constructor for the module class.  It sets default variable\n'
        defFile += '    values and initializes the various parts of the model */\n'
        defFile += self._className + '::' + self._className + '()\n'
        defFile += '{\n'
        defFile += '}\n'
        defFile += '\n'
        defFile += '/*! Module Destructor */\n'
        defFile += self._className + '::~' + self._className + '()\n'
        defFile += '{\n'
        defFile += '}\n'
        defFile += '\n'
        defFile += '/*! This method is used to reset the module and checks that required input messages are connect.\n'
        defFile += '    @return void\n'
        defFile += '*/\n'
        defFile += 'void ' + self._className + '::Reset(uint64_t CurrentSimNanos)\n'
        defFile += '{\n'
        defFile += '    // check that required input messages are connected\n'
        for msg in inMsgList:
            defFile += '    if (!this->' + msg['var'] + '.isLinked()) {\n'
            defFile += '        bskLogger.bskLog(BSK_ERROR, "' + self._className + '.' + msg['var'] + ' was not linked.");\n'
            defFile += '    }\n'
        defFile += '\n'
        defFile += '}\n'
        defFile += '\n'
        defFile += '\n'
        defFile += '/*! This is the main method that gets called every time the module is updated.  ' \
                   'Provide an appropriate description.\n'
        defFile += '    @return void\n'
        defFile += '*/\n'
        defFile += 'void ' + self._className + '::UpdateState(uint64_t CurrentSimNanos)\n'
        defFile += '{\n'
        for msg in inMsgList + outMsgList:
            defFile += '    ' + msg['type'] + 'Payload ' + msg['var'] + 'Buffer;  //!< local copy of message buffer\n'
        defFile += '\n'
        defFile += '    // always zero the output message buffers before assigning values\n'
        for msg in outMsgList:
            defFile += '    ' + msg['var'] + 'Buffer = this->' + msg['var'] + '.zeroMsgPayload;\n'
        defFile += '\n'
        defFile += '    // read in the input messages\n'
        for msg in inMsgList:
            defFile += '    ' + msg['var'] + 'Buffer = this->' + msg['var'] + '();\n'
        defFile += '\n'
        defFile += '    // do some math and stuff to populate the output messages\n'
        defFile += '\n'
        defFile += '    // write to the output messages\n'
        for msg in outMsgList:
            defFile += '    this->' + msg['var'] + '.write(&' + msg['var'] + 'Buffer, this->moduleID, CurrentSimNanos);\n'
        defFile += '}\n'
        defFile += '\n'

        with open(defFileName, 'w') as w:
            w.write(defFile)
        self.log("Done")

        #
        # make module swig interface file
        #
        swigFileName = name + ".i"
        self.log(statusColor + "Creating Swig Interface File " + swigFileName + ":" + endColor, end=" ")
        swigFile = licenseC
        swigFile += '%module ' + name + '\n'
        swigFile += '%{\n'
        swigFile += '    #include "' + name + '.h"\n'
        swigFile += '%}\n'
        swigFile += '\n'
        swigFile += '%pythoncode %{\n'
        swigFile += '    from Basilisk.architecture.swig_common_model import *\n'
        swigFile += '%}\n'
        swigFile += '%include "std_string.i"\n'
        swigFile += '%include "swig_conly_data.i"\n'
        swigFile += '\n'
        swigFile += '%include "sys_model.i"\n'
        swigFile += '%include "' + name + '.h"\n'
        swigFile += '\n'
        includedMsgs = []
        for msg in inMsgList + outMsgList:
            # ensure we don't include message definition files multiple times
            if msg['type'] not in includedMsgs:
                if msg['wrap'] == 'C':
                    swigFile += '%include "architecture/msgPayloadDefC/' + msg['type'] + 'Payload.h"\n'
                    swigFile += 'struct ' + msg['type'] + '_C;\n'
                if msg['wrap'] == 'C++':
                    swigFile += '%include "architecture/msgPayloadDefCpp/' + msg['type'] + 'Payload.h"\n'
                includedMsgs.append(msg['type'])
        swigFile += '\n'
        swigFile += '%pythoncode %{\n'
        swigFile += 'import sys\n'
        swigFile += 'protectAllClasses(sys.modules[__name__])\n'
        swigFile += '%}\n'
        swigFile += '\n'

        with open(swigFileName, 'w') as w:
            w.write(swigFile)
        self.log("Done")

        # make module definition file
        self.createRstFile()

        # make module unit test file
        self.createTestFile("C++")

        # restore current working directory
        os.chdir(initialCwd)

    def createCModule(self):
        """
        Create a C Basilisk module
        """
        modulePath = self.modulePathRelSrc
        name = self.moduleName
        briefDescription = self.briefDescription
        inMsgList = self.inMsgList
        outMsgList = self.outMsgList

        self.log(statusColor + "\nCreating C Module: " + endColor + name)
        self._className = re.sub('([a-zA-Z])', lambda x: x.groups()[0].upper(), name, 1)

        # read in the license information
        self.readLicense()
        licenseC = "/*" + self._licenseText + "*/\n\n"

        # make sure the path, specified relative to basilisk/src, to the new module location is correct
        self._absPath = os.path.join(pathToSrc, modulePath)
        self.checkPathToNewFolderLocation()

        # create new Module folder
        self._newModuleLocation = os.path.join(self._absPath, name)
        self.createNewModuleFolder()

        #
        # make module header file
        #
        headerFileName = name + ".h"
        self.log(statusColor + "Creating Header File " + headerFileName + ":" + endColor, end=" ")
        headerFile = licenseC
        headerFile += '\n'
        headerFile += '#ifndef ' + name.upper() + '_H\n'
        headerFile += '#define ' + name.upper() + '_H\n'
        headerFile += '\n'
        headerFile += '#include <stdint.h>\n'
        # loop over message definition includes
        includedMsgs = []
        for msg in inMsgList + outMsgList:
            # ensure we don't include message definition files multiple times
            if msg['type'] not in includedMsgs:
                if msg['wrap'] == 'C':
                    headerFile += '#include "cMsgCInterface/' + msg['type'] + '_C.h"\n'
                if msg['wrap'] == 'C++':
                    self.log(failColor + "Error: " + endColor + "You can't include C++ messages in a C module.")
                    exit()
                includedMsgs.append(msg['type'])
        headerFile += '#include "architecture/utilities/bskLogging.h"\n'
        headerFile += '\n'
        headerFile += '/*! @brief ' + briefDescription + '\n */\n'
        headerFile += 'typedef struct {\n'
        headerFile += '\n'
        headerFile += '    /* declare module IO interfaces */\n'
        for msg in inMsgList:
            headerFile += '    ' + msg['type'] + '_C ' + msg['var'] \
                          + ';  //!< ' + msg['desc'] + '\n'
        for msg in outMsgList:
            headerFile += '    ' + msg['type'] + '_C ' + msg['var'] \
                          + ';  //!< ' + msg['desc'] + '\n'
        headerFile += '\n'
        headerFile += '    BSKLogger *bskLogger;  //!< BSK Logging\n'
        headerFile += '}' + name + 'Config;\n'
        headerFile += '\n'
        headerFile += '#ifdef __cplusplus\n'
        headerFile += 'extern "C" {\n'
        headerFile += '#endif\n'
        headerFile += '    void SelfInit_' + name + '(' + name + 'Config *configData, int64_t moduleID);\n'
        headerFile += '    void Update_' + name + '(' + name + 'Config *configData, uint64_t callTime, int64_t moduleID);\n'
        headerFile += '    void Reset_' + name + '(' + name + 'Config *configData, uint64_t callTime, int64_t moduleID);\n'
        headerFile += '\n'
        headerFile += '#ifdef __cplusplus\n'
        headerFile += '}\n'
        headerFile += '#endif\n'
        headerFile += '\n'
        headerFile += '#endif\n'

        with open(headerFileName, 'w') as w:
            w.write(headerFile)
        self.log("Done")

        #
        # make module definition file
        #
        defFileName = name + ".c"
        self.log(statusColor + "Creating Definition File " + defFileName + ":" + endColor, end=" ")
        defFile = licenseC
        defFile += '\n'
        defFile += '#include "' + modulePath + '/' + name + '/' + name + '.h"\n'
        defFile += '#include "string.h"\n'
        defFile += '\n'
        defFile += '/*!\n'
        defFile += '    This method initializes the output messages for this module.\n'
        defFile += ' @return void\n'
        defFile += ' @param configData The configuration data associated with this module\n'
        defFile += ' @param moduleID The module identifier\n'
        defFile += ' */\n'
        defFile += 'void SelfInit_' + name + '(' + name + 'Config  *configData, int64_t moduleID)\n'
        defFile += '{\n'
        for msg in outMsgList:
            defFile += '    ' + msg['type'] + '_C_init(&configData->' + msg['var'] + ');\n'
        defFile += '}\n'
        defFile += '\n'
        defFile += '\n'
        defFile += '/*! This method performs a complete reset of the module.  Local module variables that retain\n'
        defFile += '    time varying states between function calls are reset to their default values.\n'
        defFile += '    Check if required input messages are connected.\n'
        defFile += ' @return void\n'
        defFile += ' @param configData The configuration data associated with the module\n'
        defFile += ' @param callTime [ns] time the method is called\n'
        defFile += ' @param moduleID The module identifier\n'
        defFile += '*/\n'
        defFile += 'void Reset_' + name + '(' + name + 'Config *configData, uint64_t callTime, int64_t moduleID)\n'
        defFile += '{\n'
        defFile += '    // check if the required message has not been connected\n'
        for msg in inMsgList:
            defFile += '    if (!' + msg['type'] + '_C_isLinked(&configData->' + msg['var'] + ')) {\n'
            defFile += '        _bskLog(configData->bskLogger, BSK_ERROR, "Error: ' + name + '.' + msg['var'] \
                       + ' was not connected.");\n'
            defFile += '    }\n'

        defFile += '}\n'
        defFile += '\n'
        defFile += '\n'
        defFile += '/*! Add a description of what this main Update() routine does for this module\n'
        defFile += ' @return void\n'
        defFile += ' @param configData The configuration data associated with the module\n'
        defFile += ' @param callTime The clock time at which the function was called (nanoseconds)\n'
        defFile += ' @param moduleID The module identifier\n'
        defFile += '*/\n'
        defFile += 'void Update_' + name + '(' + name + 'Config *configData, uint64_t callTime, int64_t moduleID)\n'
        defFile += '{\n'
        for msg in inMsgList + outMsgList:
            defFile += '    ' + msg['type'] + 'Payload ' + msg['var'] + 'Buffer;  //!< local copy of message buffer\n'
        defFile += '\n'
        defFile += '    // always zero the output message buffers before assigning values\n'
        for msg in outMsgList:
            defFile += '    ' + msg['var'] + 'Buffer = ' + msg['type'] + '_C_zeroMsgPayload();\n'
        defFile += '\n'
        defFile += '    // read in the input messages\n'
        for msg in inMsgList:
            defFile += '    ' + msg['var'] + 'Buffer = ' + msg['type'] + '_C_read(&configData->' + msg['var'] + ');\n'
        defFile += '\n'
        defFile += '    // do some math and stuff to populate the output messages\n'
        defFile += '\n'
        defFile += '    // write to the output messages\n'
        for msg in outMsgList:
            defFile += '    ' + msg['type'] + '_C_write(&' + msg['var'] + 'Buffer, &configData->' + msg[
                'var'] + ', moduleID, callTime);\n'
        defFile += '}\n'
        defFile += '\n'


        with open(defFileName, 'w') as w:
            w.write(defFile)
        self.log("Done")

        #
        # make module swig interface file
        #
        swigFileName = name + ".i"
        self.log(statusColor + "Creating Swig Interface File " + swigFileName + ":" + endColor, end=" ")
        swigFile = licenseC
        swigFile += '%module ' + name + '\n'
        swigFile += '%{\n'
        swigFile += '    #include "' + name + '.h"\n'
        swigFile += '%}\n'
        swigFile += '\n'
        swigFile += '%pythoncode %{\n'
        swigFile += '    from Basilisk.architecture.swig_common_model import *\n'
        swigFile += '%}\n'
        swigFile += '%include "swig_c_wrap.i"\n'
        swigFile += f'%c_wrap({name});\n'
        swigFile += '\n'
        swigFile += '%include "' + name + '.h"\n'
        swigFile += '\n'
        includedMsgs = []
        for msg in inMsgList + outMsgList:
            # ensure we don't include message definition files multiple times
            if msg['type'] not in includedMsgs:
                if msg['wrap'] == 'C':
                    swigFile += '%include "architecture/msgPayloadDefC/' + msg['type'] + 'Payload.h"\n'
                    swigFile += 'struct ' + msg['type'] + '_C;\n'
                if msg['wrap'] == 'C++':
                    self.log(failColor + "ERROR: " + endColor + 'you cannot swig a C++ message in a C module.')
                includedMsgs.append(msg['type'])
        swigFile += '\n'
        swigFile += '%pythoncode %{\n'
        swigFile += 'import sys\n'
        swigFile += 'protectAllClasses(sys.modules[__name__])\n'
        swigFile += '%}\n'
        swigFile += '\n'

        with open(swigFileName, 'w') as w:
            w.write(swigFile)
        self.log("Done")

        # make module definition file
        self.createRstFile()

        # make module unit test file
        self.createTestFile("C")

        os.chdir(initialCwd)


def fillCppInfo(module):
    """Fill in the C++ module information.  This should be edited before running to meet the new module needs."""
    # define the path where the Basilisk module folder will be
    module.modulePathRelSrc = os.path.join("moduleTemplates", "")

    # define module name and brief description
    module.moduleName = "autoCppModule"        # should be lower camel case
    module.briefDescription = "This is an auto-created sample C++ module.  The description is included with " \
        "the module class definition"
    module.copyrightHolder = "Autonomous Vehicle Systems Lab, University of Colorado Boulder"

    # provide list of input messages
    # leave list empty if there are no input messages
    inMsgList = list()
    inMsgList.append({'type': 'AttRefMsg', 'var': 'someInMsg', 'desc': 'input msg description', 'wrap': 'C'})
    inMsgList.append({'type': 'AttRefMsg', 'var': 'some2InMsg', 'desc': 'input msg description', 'wrap': 'C'})
    inMsgList.append({'type': 'CSSConfigMsg', 'var': 'anotherInMsg', 'desc': 'input msg description', 'wrap': 'C'})
    inMsgList.append({'type': 'CSSConfigLogMsg', 'var': 'anotherCppInMsg', 'desc': 'input msg description', 'wrap': 'C++'})
    module.inMsgList = inMsgList

    # provide list of output messages
    # leave list empty if there are no input messages
    outMsgList = list()
    outMsgList.append({'type': 'AttRefMsg', 'var': 'some2OutMsg', 'desc': 'output msg description', 'wrap': 'C'})
    outMsgList.append({'type': 'SCStatesMsg', 'var': 'someOutMsg', 'desc': 'output msg description', 'wrap': 'C'})
    outMsgList.append({'type': 'RWConfigMsg', 'var': 'anotherCppOutMsg', 'desc': 'output msg description', 'wrap': 'C++'})
    module.outMsgList = outMsgList


def fillCInfo(module):
    """Fill in the C module information.  This should be edited before running to meet the new module needs."""
    # define the path where the Basilisk module folder will be
    module.modulePathRelSrc = os.path.join("moduleTemplates", "")

    # define module name and brief description
    module.moduleName = "autoCModule"        # should be lower camel case
    module.briefDescription = "This is an auto-created sample C module.  The description is included with " \
        "the module class definition"
    module.copyrightHolder = "Autonomous Vehicle Systems Lab, University of Colorado Boulder"

    # provide list of input messages
    # leave list empty if there are no input messages
    inMsgList = list()
    inMsgList.append({'type': 'AttRefMsg', 'var': 'someInMsg', 'desc': 'input msg description', 'wrap': 'C'})
    inMsgList.append({'type': 'AttRefMsg', 'var': 'some2InMsg', 'desc': 'input msg description', 'wrap': 'C'})
    inMsgList.append({'type': 'CSSConfigMsg', 'var': 'anotherInMsg', 'desc': 'input msg description', 'wrap': 'C'})
    module.inMsgList = inMsgList

    # provide list of output messages
    # leave list empty if there are no input messages
    outMsgList = list()
    outMsgList.append({'type': 'AttRefMsg', 'var': 'some2OutMsg', 'desc': 'output msg description', 'wrap': 'C'})
    outMsgList.append({'type': 'SCStatesMsg', 'var': 'someOutMsg', 'desc': 'output msg description', 'wrap': 'C'})
    module.outMsgList = outMsgList


if __name__ == "__main__":
    makeModule = moduleGenerator()

    fillCppInfo(makeModule)
    makeModule.createCppModule()

    fillCInfo(makeModule)
    makeModule.createCModule()