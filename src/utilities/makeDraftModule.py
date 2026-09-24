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

import keyword
import os
import re
import shutil
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path
from tempfile import TemporaryDirectory, mkdtemp

# assumes this script is in .../basilisk/src/utilities
pathToSrc = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

statusColor = '\033[92m'
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
        self.variableList = []  # list of module variables

        # module behavior flags
        self.cleanBuild = False  # flag if any prior directories should be deleted automatically
        self.verbose = True  # flag if the status messages should be printed

        # private class variables
        self._absPath = None  # absolute path to the folder which will contain the module folder
        self._newModuleLocation = None  # absolute path to the auto-generated Basilisk module folder
        self._licenseText = None  # BSK open-source license statement
        self._output_path = None  # temporary directory used while generating files

    def log(self, statement, **kwargs):
        if self.verbose:
            if 'end' in kwargs:
                endString = kwargs['end']
                print(statement, end=endString)
            else:
                print(statement)

    def checkPathToNewFolderLocation(self):
        """Check the destination parent without changing the working directory."""
        self.log(f"{statusColor}Checking Module location:{endColor}", end=" ")
        if not self._absPath.is_dir():
            raise NotADirectoryError(f"Incorrect path to the new folder: {self._absPath}")
        self.log("Done")
        self.log(self._absPath)

    def _validate_specification(self, module_type):
        """Validate generation inputs before creating or replacing any files."""
        for field in ("moduleName", "briefDescription", "copyrightHolder"):
            value = getattr(self, field)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{field} must be a nonempty string")
        self._validate_identifier(self.moduleName, "moduleName")

        if not isinstance(self.modulePathRelSrc, (str, os.PathLike)):
            raise ValueError("modulePathRelSrc must be a path relative to basilisk/src")
        module_path = Path(self.modulePathRelSrc)
        if module_path.is_absolute() or not module_path.parts or ".." in module_path.parts:
            raise ValueError("modulePathRelSrc must stay within a package under basilisk/src")
        self._validate_identifier(module_path.parts[0], "Basilisk package name")
        source_path = Path(pathToSrc).resolve()
        self._absPath = (source_path / module_path).resolve()
        if not self._absPath.is_relative_to(source_path):
            raise ValueError("modulePathRelSrc must stay within basilisk/src")
        self.checkPathToNewFolderLocation()
        self._newModuleLocation = self._absPath / self.moduleName
        self._module_path = module_path
        # Basilisk flattens modules below each top-level source package.
        self._python_package = module_path.parts[0]

        names = set()
        message_wrappers = {}
        for field in ("inMsgList", "outMsgList", "variableList"):
            entries = getattr(self, field)
            if not isinstance(entries, list):
                raise ValueError(f"{field} must be a list")
            for entry in entries:
                required = ("type", "var", "desc")
                if field != "variableList":
                    required += ("wrap",)
                if not isinstance(entry, dict) or any(
                    not isinstance(entry.get(key), str) for key in required
                ):
                    raise ValueError(f"{field} entries require string fields: {', '.join(required)}")
                self._validate_identifier(entry["var"], f"{field} variable")
                if entry["var"] in names:
                    raise ValueError(f"Duplicate module variable: {entry['var']}")
                names.add(entry["var"])
                if not entry["type"].strip():
                    raise ValueError(f"{field} type must not be empty")
                if field != "variableList":
                    self._validate_identifier(entry["type"], "message type")
                    allowed_wrappers = ("C",) if module_type == "C" else ("C", "C++")
                    if entry["wrap"] not in allowed_wrappers:
                        raise ValueError(f"{module_type} modules require message wrappers in {allowed_wrappers}")
                    previous = message_wrappers.setdefault(entry["type"], entry["wrap"])
                    if previous != entry["wrap"]:
                        raise ValueError(f"Conflicting wrappers for message type: {entry['type']}")

    @staticmethod
    def _validate_identifier(value, description):
        """Reject names that cannot be used as generated identifiers or filenames."""
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", value) or keyword.iskeyword(value):
            raise ValueError(f"Invalid {description}: {value!r}")

    def _check_destination(self):
        """Return whether replacement of an existing module is authorized."""
        destination = self._newModuleLocation
        if destination.is_symlink():
            raise ValueError(f"The module destination must not be a symbolic link: {destination}")
        if not destination.exists():
            return False
        if not destination.is_dir():
            raise FileExistsError(f"The module destination is not a directory: {destination}")
        self.log(f"{warningColor}WARNING: {endColor}The new module destination already exists.")
        if not self.cleanBuild and input("Do you want to replace this folder? (y or n): ") != "y":
            raise FileExistsError(f"Module creation cancelled; preserved {destination}")
        return True

    @contextmanager
    def _module_directory(self, module_type):
        """Stage a complete draft before publishing it at the requested destination."""
        self._validate_specification(module_type)
        self.readLicense()
        replace_existing = self._check_destination()
        with TemporaryDirectory(prefix=f".{self.moduleName}-draft-", dir=self._absPath) as directory:
            self._output_path = Path(directory) / self.moduleName
            self._output_path.mkdir()
            try:
                yield
                self._publish_module(replace_existing)
            finally:
                self._output_path = None

    def _publish_module(self, replace_existing):
        """Publish staged files, restoring the old directory if installation fails."""
        destination = self._newModuleLocation
        backup_root = None
        if destination.is_symlink():
            raise ValueError(f"The module destination must not be a symbolic link: {destination}")
        if destination.exists():
            if not replace_existing or not destination.is_dir():
                raise FileExistsError(f"The module destination already exists: {destination}")
            # Keep the backup outside staging so failed recovery cannot delete it.
            backup_root = Path(mkdtemp(prefix=f".{self.moduleName}-backup-", dir=self._absPath))
            try:
                destination.rename(backup_root / self.moduleName)
            except BaseException:
                backup_root.rmdir()
                raise
        try:
            self._output_path.rename(destination)
        except BaseException:
            if backup_root is not None:
                backup = backup_root / self.moduleName
                try:
                    backup.rename(destination)
                except OSError as error:
                    raise OSError(f"Could not restore the original module; its files remain at {backup}") from error
                backup_root.rmdir()
            raise
        if backup_root is not None:
            shutil.rmtree(backup_root)

    def readLicense(self):
        """Read the Basilisk license file"""
        self.log(statusColor + "Importing License:" + endColor, end=" ")
        with (Path(pathToSrc).parent / "LICENSE").open(encoding="utf-8") as f:
            self._licenseText = f.read()
            self._licenseText = self._licenseText.replace("2016", str(datetime.now().year))
            self._licenseText = self._licenseText.replace(
                "Autonomous Vehicle Systems Lab, University of Colorado at Boulder",
                self.copyrightHolder)
        self.log("Done")

    def createRstFile(self, moduleType):
        """Create the Module RST documentation draft."""
        rstFileName = f"{self.moduleName}.rst"
        self.log(f"{statusColor}Creating RST Documentation File {rstFileName}:{endColor}", end=" ")
        rstFile = 'Executive Summary\n'
        rstFile += '-----------------\n'
        rstFile += f'{self.briefDescription}\n'
        rstFile += '\n'
        rstFile += 'Module Assumptions and Limitations\n'
        rstFile += '----------------------------------\n'
        rstFile += 'Describe the model assumptions, required inputs, and limits on valid use.\n'
        rstFile += '\n'
        rstFile += 'Message Connection Descriptions\n'
        rstFile += '-------------------------------\n'
        rstFile += 'The following diagram and table list all the module input and output messages.  \n'
        rstFile += 'The module message connection is set by the user from Python.  \n'
        rstFile += 'The message type contains a link to the message structure definition, while the description \n'
        rstFile += 'provides information on what this message is used for.\n'
        rstFile += '\n'
        if self.inMsgList or self.outMsgList:
            rstFile += f'.. bsk-module-io:: {self.moduleName}\n'
            rstFile += '    :caption: Module I/O Messages\n'
            rstFile += f'    :module-type: {moduleType}\n'
            rstFile += '\n'
            for msg in self.inMsgList:
                rstFile += f'    input {msg["var"]} {msg["type"]}Payload\n'
                for descLine in str(msg["desc"]).splitlines():
                    rstFile += f'        {descLine}\n'
                rstFile += '\n'
            for msg in self.outMsgList:
                rstFile += f'    output {msg["var"]} {msg["type"]}Payload\n'
                for descLine in str(msg["desc"]).splitlines():
                    rstFile += f'        {descLine}\n'
                rstFile += '\n'
        else:
            rstFile += 'This module does not define input or output messages.\n'
        rstFile += '\n'
        rstFile += 'Detailed Module Description\n'
        rstFile += '---------------------------\n'
        rstFile += 'Describe the implemented algorithm and mathematics, including units and reset behavior.\n'
        rstFile += 'This section is optional for modules whose behavior is fully explained above.\n'
        rstFile += '\n'
        rstFile += 'User Guide\n'
        rstFile += '----------\n'
        rstFile += 'Provide a runnable Python example with configuration, message connections, and expected results.\n'
        rstFile += 'Explain which variables are configuration and which are runtime state.\n'
        rstFile += f'Use ``_UnitTest/test_{self.moduleName}.py`` as a starting point for simulation setup;\n'
        rstFile += 'its smoke checks verify execution and publication, not numerical payload values.\n'
        rstFile += '\n'
        rstFile += 'See :ref:`makingModules-3` for RST authoring instructions and :ref:`cModuleTemplate`\n'
        rstFile += 'and :ref:`cppModuleTemplate` for completed module documentation examples.\n'

        with (self._output_path / rstFileName).open('w', encoding="utf-8") as w:
            w.write(rstFile)
        self.log("Done")

    def createTestFile(self, module_type):
        """Create a smoke test for scheduled execution and output message writes."""
        test_path = self._output_path / '_UnitTest'
        test_path.mkdir()
        test_file_name = f"test_{self.moduleName}.py"
        self.log(f"{statusColor}Creating Python Smoke Test {test_file_name}:{endColor}", end=" ")
        test_file = ""
        for line in self._licenseText.split('\n'):
            test_file += f'# {line}\n'
        test_file += '\n'
        test_file += 'import numpy as np\n'
        test_file += '\n'
        test_file += 'from Basilisk.architecture import messaging\n'
        test_file += f'from Basilisk.{self._python_package} import {self.moduleName} as module_under_test\n'
        test_file += 'from Basilisk.utilities import SimulationBaseClass, macros\n'
        test_file += '\n\n'
        test_file += f'def test_{self.moduleName}():\n'
        test_file += '    """Run the draft module and check its output message publication.\n'
        test_file += '\n'
        test_file += '    **Validation Test Description**\n'
        test_file += '\n'
        test_file += '    Connect blank input messages and execute three scheduled updates.\n'
        test_file += '    Check the module call count, output written status, sample times,\n'
        test_file += '    and message write times. These checks verify scheduling and messaging;\n'
        test_file += '    add numerical payload checks when the module algorithm is implemented.\n'
        test_file += '    """\n'
        test_file += '    task_name = "unitTask"\n'
        test_file += '    simulation = SimulationBaseClass.SimBaseClass()\n'
        test_file += '    time_step = macros.sec2nano(0.5)  # [ns]\n'
        test_file += '    process = simulation.CreateNewProcess("TestProcess")\n'
        test_file += '    process.addTask(simulation.CreateNewTask(task_name, time_step))\n'
        test_file += '\n'
        test_file += '    # Set up the module to be tested.\n'
        if module_type == "C++":
            test_file += f'    module = module_under_test.{self._className}()\n'
        elif module_type == "C":
            test_file += f'    module = module_under_test.{self.moduleName}()\n'
        else:
            raise ValueError(f"Unsupported module type: {module_type}")
        test_file += f'    module.ModelTag = "{self.moduleName}Tag"\n'
        test_file += '    simulation.AddModelToTask(task_name, module)\n'
        test_file += '\n'
        test_file += '    # Configure and retain blank input messages, then subscribe the module.\n'
        test_file += '    input_messages = []\n'
        for msg in self.inMsgList:
            test_file += f'    input_payload = messaging.{msg["type"]}Payload()\n'
            test_file += f'    input_message = messaging.{msg["type"]}().write(input_payload)\n'
            test_file += f'    module.{msg["var"]}.subscribeTo(input_message)\n'
            test_file += '    input_messages.append(input_message)\n'
            test_file += '\n'
        test_file += '    # Record each output after the module runs in the same task.\n'
        test_file += '    output_readers = {\n'
        for msg in self.outMsgList:
            test_file += f'        "{msg["var"]}": messaging.{msg["type"]}Reader(),\n'
        test_file += '    }\n'
        test_file += '    output_recorders = {}\n'
        test_file += '    for name, reader in output_readers.items():\n'
        test_file += '        reader.subscribeTo(getattr(module, name))\n'
        test_file += '        recorder = reader.recorder()\n'
        test_file += '        output_recorders[name] = recorder\n'
        test_file += '        simulation.AddModelToTask(task_name, recorder)\n'
        test_file += '\n'
        test_file += '    simulation.InitializeSimulation()\n'
        test_file += '    simulation.ConfigureStopTime(2 * time_step)\n'
        test_file += '    simulation.ExecuteSimulation()\n'
        test_file += '\n'
        test_file += '    expected_times = np.array([0, time_step, 2 * time_step], dtype=np.uint64)  # [ns]\n'
        test_file += '    assert module.CallCounts == len(expected_times), "Module did not run three times"\n'
        test_file += '    for name, recorder in output_recorders.items():\n'
        test_file += '        assert output_readers[name].isWritten(), f"{name} was never written"\n'
        test_file += '        np.testing.assert_array_equal(\n'
        test_file += '            recorder.times(), expected_times, err_msg=f"{name} recording times"\n'
        test_file += '        )\n'
        test_file += '        np.testing.assert_array_equal(\n'
        test_file += '            recorder.timesWritten(), expected_times, err_msg=f"{name} write times"\n'
        test_file += '        )\n'
        test_file += '\n'
        test_file += '    # Add module-specific numerical payload checks here.\n'
        test_file += '\n\n'
        test_file += 'if __name__ == "__main__":\n'
        test_file += f'    test_{self.moduleName}()\n'

        with (test_path / test_file_name).open('w', encoding="utf-8") as output:
            output.write(test_file)
        self.log("Done")

    def createCppModule(self):
        """Create a C++ draft, preserving existing files if generation fails."""
        with self._module_directory("C++"):
            self._create_cpp_module()

    def _create_cpp_module(self):
        """Write the C++ draft into the staging directory."""
        modulePath = self._module_path.as_posix()
        name = self.moduleName
        briefDescription = self.briefDescription
        inMsgList = self.inMsgList
        outMsgList = self.outMsgList
        variableList = self.variableList

        self.log(statusColor + '\nCreating C++ Module: ' + endColor + name)
        self._className = re.sub('([a-zA-Z])', lambda x: x.groups()[0].upper(), name, count=1)

        licenseC = "/*" + self._licenseText + "*/\n\n"

        #
        # make module header file
        #
        headerFileName = name + ".h"
        self.log(f"{statusColor}Creating Header File {headerFileName}:{endColor}", end=" ")
        headerFile = licenseC
        headerFile += '\n'
        headerFile += f'#ifndef {name.upper()}_H\n'
        headerFile += f'#define {name.upper()}_H\n'
        headerFile += '\n'
        headerFile += '#include "architecture/_GeneralModuleFiles/sys_model.h"\n'
        # loop over message definition includes
        includedMsgs = []
        for msg in inMsgList + outMsgList:
            # ensure we don't include message definition files multiple times
            if msg['type'] not in includedMsgs:
                if msg['wrap'] == 'C':
                    headerFile += f'#include "architecture/msgPayloadDefC/{msg["type"]}Payload.h"\n'
                if msg['wrap'] == 'C++':
                    headerFile += f'#include "architecture/msgPayloadDefCpp/{msg["type"]}Payload.h"\n'
                includedMsgs.append(msg['type'])
        headerFile += '#include "architecture/utilities/bskLogging.h"\n'
        headerFile += '#include "architecture/messaging/messaging.h"\n'
        headerFile += '\n'
        headerFile += f'/*! @brief {briefDescription}\n */\n'
        headerFile += f'class {self._className}: public SysModel {{\n'
        headerFile += 'public:\n'
        headerFile += f'    {self._className}();\n'
        headerFile += f'    ~{self._className}() = default;\n'
        headerFile += '\n'
        headerFile += '    void Reset(uint64_t CurrentSimNanos);\n'
        headerFile += '    void UpdateState(uint64_t CurrentSimNanos);\n'
        headerFile += '\n'
        headerFile += 'public:\n'
        for msg in inMsgList:
            headerFile += f'    ReadFunctor<{msg["type"]}Payload> {msg["var"]};  //!< {msg["desc"]}\n'
        headerFile += '\n'
        for msg in outMsgList:
            headerFile += f'    Message<{msg["type"]}Payload> {msg["var"]};  //!< {msg["desc"]}\n'
        headerFile += '\n'
        headerFile += '    BSKLogger bskLogger;              //!< BSK Logging\n'
        headerFile += '\n'
        if len(variableList):
            for msg in variableList:
                varName = msg['var'];
                headerFile += f"    /** setter for `{varName}` property */\n"
                headerFile += f'    void set{varName[:1].upper() + varName[1:]}({msg["type"]});\n'
                headerFile += f"    /** getter for `{varName}` property */\n"
                headerFile += (f'    {msg["type"]} get{varName[:1].upper() + varName[1:]}() '
                               f'const {{return this->{varName};}}\n')
            headerFile += '\n'
            headerFile += 'private:\n'
            for msg in variableList:
                headerFile += f'    {msg["type"]} {msg["var"]};  //!< {msg["desc"]}\n'
        headerFile += '\n'
        headerFile += '};\n'
        headerFile += '\n'
        headerFile += "\n#endif\n"

        with (self._output_path / headerFileName).open('w', encoding="utf-8") as w:
            w.write(headerFile)
        self.log("Done")

        #
        # make module definition file
        #
        defFileName = name + ".cpp"
        self.log(statusColor + "Creating Definition File " + defFileName + ":" + endColor, end=" ")
        defFile = licenseC
        defFile += '\n'
        defFile += f'#include "{modulePath}/{name}/{name}.h"\n'
        defFile += '#include <iostream>\n'
        defFile += '#include <cstring>\n'
        defFile += '\n'
        defFile += '/*! This is the constructor for the module class.  It sets default variable\n'
        defFile += '    values and initializes the various parts of the model */\n'
        defFile += self._className + '::' + self._className + '()\n'
        defFile += '{\n'
        if self.variableList:
            defFile += '    // initialize module variables\n'
            for msg in variableList:
                defFile += f'    this->{msg["var"]} = {{}};\n'

        defFile += '}\n'
        defFile += '\n'
        defFile += '/*! This method is used to reset the module and checks that required input messages are connected.\n'
        defFile += '*/\n'
        defFile += f'void {self._className}::Reset(uint64_t CurrentSimNanos [[maybe_unused]])\n'
        defFile += '{\n'
        defFile += '    // check that required input messages are connected\n'
        for msg in inMsgList:
            defFile += f'    if (!this->{msg["var"]}.isLinked()) {{\n'
            defFile += f'        bskLogger.bskError("{self._className}.{msg["var"]} was not linked.");\n'
            defFile += '    }\n'
        defFile += '\n'
        defFile += '}\n'
        defFile += '\n'
        defFile += '\n'
        defFile += '/*! This is the main method that gets called every time the module is updated.  ' \
                   'Provide an appropriate description.\n'
        defFile += '*/\n'
        defFile += f'void {self._className}::UpdateState(uint64_t CurrentSimNanos)\n'
        defFile += '{\n'
        for msg in inMsgList + outMsgList:
            defFile += f'    {msg["type"]}Payload {msg["var"]}Buffer;  //!< local copy of message buffer\n'
        defFile += '\n'
        defFile += '    // always zero the output message buffers before assigning values\n'
        for msg in outMsgList:
            defFile += f'    {msg["var"]}Buffer = this->{msg["var"]}.zeroMsgPayload;\n'
        defFile += '\n'
        defFile += '    // read in the input messages\n'
        for msg in inMsgList:
            defFile += f'    {msg["var"]}Buffer = this->{msg["var"]}();\n'
        defFile += '\n'
        defFile += '    // do some math and stuff to populate the output messages\n'
        for msg in inMsgList:
            defFile += f'    (void) {msg["var"]}Buffer;\n'
        defFile += '\n'
        defFile += '    // write to the output messages\n'
        for msg in outMsgList:
            defFile += f'    this->{msg["var"]}.write(&{msg["var"]}Buffer, this->moduleID, CurrentSimNanos);\n'
        defFile += '}\n'
        defFile += '\n'

        if variableList:
            for msg in variableList:
                varName = msg['var']
                defFile += f'void {self._className}::set{varName[:1].upper() + varName[1:]}({msg["type"]} var)\n'
                defFile += '{\n'
                defFile += f'    this->{varName} = var;\n'
                defFile += '}\n'
                defFile += '\n'

        with (self._output_path / defFileName).open('w', encoding="utf-8") as w:
            w.write(defFile)
        self.log("Done")

        #
        # make module swig interface file
        #
        swigFileName = name + ".i"
        self.log(statusColor + "Creating Swig Interface File " + swigFileName + ":" + endColor, end=" ")
        swigFile = licenseC
        swigFile += f'%module {name}\n'
        swigFile += '\n'
        swigFile += '%include "architecture/utilities/bskException.swg"\n'
        swigFile += '%default_bsk_exception();\n'
        swigFile += '\n'
        swigFile += '%{\n'
        swigFile += f'    #include "{name}.h"\n'
        swigFile += '%}\n'
        swigFile += '\n'
        swigFile += '%pythoncode %{\n'
        swigFile += '    from Basilisk.architecture.swig_common_model import *\n'
        swigFile += '%}\n'
        swigFile += '%include "std_string.i"\n'
        swigFile += '%include "swig_conly_data.i"\n'
        swigFile += '\n'
        swigFile += '%include "sys_model.i"\n'
        swigFile += f'%include "{name}.h"\n'
        swigFile += '\n'
        includedMsgs = []
        for msg in inMsgList + outMsgList:
            # ensure we don't include message definition files multiple times
            if msg['type'] not in includedMsgs:
                if msg['wrap'] == 'C':
                    swigFile += f'%include "architecture/msgPayloadDefC/{msg["type"]}Payload.h"\n'
                    swigFile += f'struct {msg["type"]}_C;\n'
                if msg['wrap'] == 'C++':
                    swigFile += f'%include "architecture/msgPayloadDefCpp/{msg["type"]}Payload.h"\n'
                includedMsgs.append(msg['type'])
        swigFile += '\n'
        swigFile += '%pythoncode %{\n'
        swigFile += 'import sys\n'
        swigFile += 'protectAllClasses(sys.modules[__name__])\n'
        swigFile += '%}\n'
        swigFile += '\n'

        with (self._output_path / swigFileName).open('w', encoding="utf-8") as w:
            w.write(swigFile)
        self.log("Done")

        # make module definition file
        self.createRstFile("C++")

        # make module unit test file
        self.createTestFile("C++")

    def createCModule(self):
        """Create a C draft, preserving existing files if generation fails."""
        with self._module_directory("C"):
            self._create_c_module()

    def _create_c_module(self):
        """Write the C draft into the staging directory."""
        modulePath = self._module_path.as_posix()
        name = self.moduleName
        briefDescription = self.briefDescription
        inMsgList = self.inMsgList
        outMsgList = self.outMsgList
        variableList = self.variableList

        self.log(f"{statusColor}\nCreating C Module: {endColor}{name}")
        self._className = re.sub('([a-zA-Z])', lambda x: x.groups()[0].upper(), name, count=1)

        licenseC = f"/*{self._licenseText}*/\n\n"

        #
        # make module header file
        #
        headerFileName = f"{name}.h"
        self.log(f"{statusColor}Creating Header File {headerFileName}:{endColor}", end=" ")
        headerFile = licenseC
        headerFile += '\n'
        headerFile += f'#ifndef {name.upper()}_H\n'
        headerFile += f'#define {name.upper()}_H\n'
        headerFile += '\n'
        headerFile += '#include <stdint.h>\n'
        # loop over message definition includes
        includedMsgs = []
        for msg in inMsgList + outMsgList:
            # ensure we don't include message definition files multiple times
            if msg['type'] not in includedMsgs:
                if msg['wrap'] == 'C':
                    headerFile += f'#include "cMsgCInterface/{msg["type"]}_C.h"\n'
                includedMsgs.append(msg['type'])
        headerFile += '#include "architecture/utilities/bskLogging.h"\n'
        headerFile += '\n'
        headerFile += f'/*! @brief {briefDescription}\n */\n'
        headerFile += f'typedef struct {{\n'
        headerFile += '\n'
        headerFile += '    /* declare module IO interfaces */\n'
        for msg in inMsgList:
            headerFile += f'    {msg["type"]}_C {msg["var"]};  //!< {msg["desc"]}\n'
        for msg in outMsgList:
            headerFile += f'    {msg["type"]}_C {msg["var"]};  //!< {msg["desc"]}\n'
        if len(variableList):
            headerFile += '\n'
            for msg in variableList:
                headerFile += f'    {msg["type"]} {msg["var"]};  //!< {msg["desc"]}\n'
        headerFile += '\n'
        headerFile += '    BSKLogger *bskLogger;  //!< BSK Logging\n'
        headerFile += f'}}{name}Config;\n'
        headerFile += '\n'
        headerFile += '#ifdef __cplusplus\n'
        headerFile += 'extern "C" {\n'
        headerFile += '#endif\n'
        headerFile += f'    void SelfInit_{name}({name}Config *configData, int64_t moduleID);\n'
        headerFile += f'    void Update_{name}({name}Config *configData, uint64_t callTime, int64_t moduleID);\n'
        headerFile += f'    void Reset_{name}({name}Config *configData, uint64_t callTime, int64_t moduleID);\n'
        headerFile += '\n'
        headerFile += '#ifdef __cplusplus\n'
        headerFile += '}\n'
        headerFile += '#endif\n'
        headerFile += '\n'
        headerFile += '#endif\n'

        with (self._output_path / headerFileName).open('w', encoding="utf-8") as w:
            w.write(headerFile)
        self.log("Done")

        #
        # make module definition file
        #
        defFileName = f"{name}.c"
        self.log(f"{statusColor}Creating Definition File {defFileName}:{endColor}", end=" ")
        defFile = licenseC
        defFile += '\n'
        defFile += f'#include "{modulePath}/{name}/{name}.h"\n'
        defFile += '#include "string.h"\n'
        defFile += '\n'
        defFile += '/*!\n'
        defFile += '    This method initializes the output messages for this module.\n'
        defFile += '\n'
        defFile += ' @param configData The configuration data associated with this module\n'
        defFile += ' @param moduleID The module identifier\n'
        defFile += ' */\n'
        defFile += f'void SelfInit_{name}({name}Config  *configData, int64_t moduleID)\n'
        defFile += '{\n'
        defFile += '    (void) moduleID;\n'
        defFile += '\n'
        for msg in outMsgList:
            defFile += f'    {msg["type"]}_C_init(&configData->{msg["var"]});\n'
        defFile += '}\n'
        defFile += '\n'
        defFile += '\n'
        defFile += '/*! This method performs a complete reset of the module.  Local module variables that retain\n'
        defFile += '    time varying states between function calls are reset to their default values.\n'
        defFile += '    Check if required input messages are connected.\n'
        defFile += '\n'
        defFile += ' @param configData The configuration data associated with the module\n'
        defFile += ' @param callTime [ns] time the method is called\n'
        defFile += ' @param moduleID The module identifier\n'
        defFile += '*/\n'
        defFile += f'void Reset_{name}({name}Config *configData, uint64_t callTime, int64_t moduleID)\n'
        defFile += '{\n'
        defFile += '    (void) callTime;\n'
        defFile += '    (void) moduleID;\n'
        defFile += '\n'
        defFile += '    // check if the required message has not been connected\n'
        for msg in inMsgList:
            defFile += f'    if (!{msg["type"]}_C_isLinked(&configData->{msg["var"]})) {{\n'
            defFile += f'        _bskError(configData->bskLogger, "Error: {name}.{msg["var"]}' \
                       + ' was not connected.");\n'
            defFile += '    }\n'

        defFile += '}\n'
        defFile += '\n'
        defFile += '\n'
        defFile += '/*! Add a description of what this main Update() routine does for this module\n'
        defFile += '\n'
        defFile += ' @param configData The configuration data associated with the module\n'
        defFile += ' @param callTime The clock time at which the function was called (nanoseconds)\n'
        defFile += ' @param moduleID The module identifier\n'
        defFile += '*/\n'
        defFile += f'void Update_{name}({name}Config *configData, uint64_t callTime, int64_t moduleID)\n'
        defFile += '{\n'
        for msg in inMsgList + outMsgList:
            defFile += f'    {msg["type"]}Payload {msg["var"]}Buffer;  //!< local copy of message buffer\n'
        defFile += '\n'
        defFile += '    // always zero the output message buffers before assigning values\n'
        for msg in outMsgList:
            defFile += f'    {msg["var"]}Buffer = {msg["type"]}_C_zeroMsgPayload();\n'
        defFile += '\n'
        defFile += '    // read in the input messages\n'
        for msg in inMsgList:
            defFile += f'    {msg["var"]}Buffer = {msg["type"]}_C_read(&configData->{msg["var"]});\n'
        defFile += '\n'
        defFile += '    // do some math and stuff to populate the output messages\n'
        for msg in inMsgList:
            defFile += f'    (void) {msg["var"]}Buffer;\n'
        defFile += '\n'
        defFile += '    // write to the output messages\n'
        for msg in outMsgList:
            defFile += f'    {msg["type"]}_C_write(&{msg["var"]}Buffer, &configData->{msg["var"]}, moduleID, callTime);\n'
        defFile += '}\n'
        defFile += '\n'

        with (self._output_path / defFileName).open('w', encoding="utf-8") as w:
            w.write(defFile)
        self.log("Done")

        #
        # make module swig interface file
        #
        swigFileName = f"{name}.i"
        self.log(f"{statusColor}Creating Swig Interface File {swigFileName}:{endColor}", end=" ")
        swigFile = licenseC
        swigFile += f'%module {name}\n'
        swigFile += '\n'
        swigFile += '%include "architecture/utilities/bskException.swg"\n'
        swigFile += '%default_bsk_exception();\n'
        swigFile += '\n'
        swigFile += '%{\n'
        swigFile += f'    #include "{name}.h"\n'
        swigFile += '%}\n'
        swigFile += '\n'
        swigFile += '%pythoncode %{\n'
        swigFile += '    from Basilisk.architecture.swig_common_model import *\n'
        swigFile += '%}\n'
        swigFile += '%include "swig_c_wrap.i"\n'
        swigFile += f'%c_wrap({name});\n'
        swigFile += '\n'
        swigFile += f'%include "{name}.h"\n'
        swigFile += '\n'
        includedMsgs = []
        for msg in inMsgList + outMsgList:
            # ensure we don't include message definition files multiple times
            if msg['type'] not in includedMsgs:
                if msg['wrap'] == 'C':
                    swigFile += f'%include "architecture/msgPayloadDefC/{msg["type"]}Payload.h"\n'
                    swigFile += f'struct {msg["type"]}_C;\n'
                includedMsgs.append(msg['type'])
        swigFile += '\n'
        swigFile += '%pythoncode %{\n'
        swigFile += 'import sys\n'
        swigFile += 'protectAllClasses(sys.modules[__name__])\n'
        swigFile += '%}\n'
        swigFile += '\n'

        with (self._output_path / swigFileName).open('w', encoding="utf-8") as w:
            w.write(swigFile)
        self.log("Done")

        # make module definition file
        self.createRstFile("C")

        # make module unit test file
        self.createTestFile("C")


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
    inMsgList.append({'type': 'AttRefMsg', 'var': 'someInMsg', 'desc': 'Input message description.', 'wrap': 'C'})
    inMsgList.append({'type': 'AttRefMsg', 'var': 'some2InMsg', 'desc': 'Input message description.', 'wrap': 'C'})
    inMsgList.append({'type': 'CSSConfigMsg', 'var': 'anotherInMsg', 'desc': 'Input message description.', 'wrap': 'C'})
    inMsgList.append({'type': 'CSSConfigLogMsg', 'var': 'anotherCppInMsg', 'desc': 'Input message description.', 'wrap': 'C++'})
    module.inMsgList = inMsgList

    # provide list of output messages
    # leave list empty if there are no input messages
    outMsgList = list()
    outMsgList.append({'type': 'AttRefMsg', 'var': 'some2OutMsg', 'desc': 'Output message description.', 'wrap': 'C'})
    outMsgList.append({'type': 'SCStatesMsg', 'var': 'someOutMsg', 'desc': 'Output message description.', 'wrap': 'C'})
    outMsgList.append({'type': 'DataStorageStatusMsg', 'var': 'anotherCppOutMsg', 'desc': 'Output message description.', 'wrap': 'C++'})
    module.outMsgList = outMsgList

    # provide list of module variables
    # leave list empty if you are not setting up module variables at this stage
    variableList = list()
    variableList.append({'type': 'double', 'var': 'varDouble', 'desc': '[units] variable description'})
    variableList.append({'type': 'int', 'var': 'varInt', 'desc': '[units] variable description'})
    module.variableList = variableList

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
    inMsgList.append({'type': 'AttRefMsg', 'var': 'someInMsg', 'desc': 'Input message description.', 'wrap': 'C'})
    inMsgList.append({'type': 'AttRefMsg', 'var': 'some2InMsg', 'desc': 'Input message description.', 'wrap': 'C'})
    inMsgList.append({'type': 'CSSConfigMsg', 'var': 'anotherInMsg', 'desc': 'Input message description.', 'wrap': 'C'})
    module.inMsgList = inMsgList

    # provide list of output messages
    # leave list empty if there are no input messages
    outMsgList = list()
    outMsgList.append({'type': 'AttRefMsg', 'var': 'some2OutMsg', 'desc': 'Output message description.', 'wrap': 'C'})
    outMsgList.append({'type': 'SCStatesMsg', 'var': 'someOutMsg', 'desc': 'Output message description.', 'wrap': 'C'})
    module.outMsgList = outMsgList

    # provide list of module variables
    # leave list empty if you are not setting up module variables at this stage
    variableList = list()
    variableList.append({'type': 'double', 'var': 'varDouble', 'desc': '[units] variable description'})
    variableList.append({'type': 'int', 'var': 'varInt', 'desc': '[units] variable description'})
    module.variableList = variableList


if __name__ == "__main__":
    makeModule = moduleGenerator()

    fillCppInfo(makeModule)
    makeModule.createCppModule()

    fillCInfo(makeModule)
    makeModule.createCModule()
