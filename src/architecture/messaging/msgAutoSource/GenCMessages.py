# Generate C code messages that are compatible with cpp functor based system
import parse
import os,errno
import shutil
import argparse
from sys import platform


class GenerateMessages:

    def __init__(self, pathToExternalModule):
        self.messageTemplate = ""
        self.headerTemplate = ""
        self.autoSourceDestDir = '../../../../dist3/autoSource/'
        self.destinationDir = os.path.join(self.autoSourceDestDir, 'cMsgCInterface/')
        self.pathToExternalModule = pathToExternalModule
        with open('./cMsgCInterfacePy.i.in', 'r') as f:
            self.swig_template_block = f.read()
        self.swigTemplate = ""
        self.messagingAutoData = list()


    def __createMessageAndHeaderTemplate(self):
        licenseREADME = list()
        with open("../../../../LICENSE", 'r') as f:
            licenseREADME.extend(["/*", f.read(),"*/\n\n"])
        with open('./README.in', 'r') as r:
            licenseREADME.append(r.read())
        self.messageTemplate = ''.join(licenseREADME)
        self.headerTemplate = ''.join(licenseREADME)
        with open('./msg_C.cpp.in', 'r') as f:
            self.messageTemplate += f.read()
        with open('./msg_C.h.in', 'r') as f:
            self.headerTemplate += f.read()

    def __recreateDestinationDirectory(self):
        if os.path.exists(self.autoSourceDestDir):
            shutil.rmtree(self.autoSourceDestDir, ignore_errors=True)
        try:
            os.makedirs(os.path.dirname(self.autoSourceDestDir))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
        print(self.destinationDir)
        os.makedirs(os.path.dirname(self.destinationDir))

    def __generateMessagingHeaderInterface(self):
        messaging_header_i_template = ""
        if platform == "linux" or platform == "linux2":
            messaging_header_i_template = "#define SWIGWORDSIZE64\n"
        with open(self.autoSourceDestDir + 'messaging.header.auto.i', 'w') as w:
            w.write(messaging_header_i_template)

    def __createMessageC(self,parentPath, external=False):
        if not external:
            messaging_i_template = "//C messages:"
        else:
            messaging_i_template = ""
        for file in os.listdir(f"{parentPath}/msgPayloadDefC"):
            if file.endswith(".h"):
                msgName = (os.path.splitext(file)[0])[:-7]
                if external:
                    relativePath = os.path.relpath(self.pathToExternalModule, "../../../architecture").replace("\\",
                                                                                                               "/")
                    messaging_i_template += f"\nINSTANTIATE_TEMPLATES({msgName}, {msgName}Payload, {relativePath}/msgPayloadDefC)"
                else:
                    messaging_i_template += f"\nINSTANTIATE_TEMPLATES({msgName}, {msgName}Payload, msgPayloadDefC)"
        with open(self.autoSourceDestDir + 'messaging.auto.i', 'a') as w:
            w.write(messaging_i_template)

    def __createMessageCpp(self,parentPath, external=False):
        if external:
            messaging_i_template = ""
        else:
            messaging_i_template = "\n\n//C++ messages:"
        for file in os.listdir(f"{parentPath}/msgPayloadDefCpp"):
            if file.endswith(".h"):
                msgName = (os.path.splitext(file)[0])[:-7]
                if external:
                    relativePath = os.path.relpath(self.pathToExternalModule, "../../../architecture").replace("\\",
                                                                                                               "/")
                    messaging_i_template += f"\nINSTANTIATE_TEMPLATES({msgName}, {msgName}Payload, {relativePath}/msgPayloadDefCpp)"
                else:
                    messaging_i_template += f"\nINSTANTIATE_TEMPLATES({msgName}, {msgName}Payload, msgPayloadDefCpp)"
        with open(self.autoSourceDestDir + 'messaging.auto.i', 'a') as w:
            w.write(messaging_i_template)

    def __generateMessages(self):
        # append all C msg definitions to the dist3/autoSource/messaging.auto.i file that is imported into messaging.auto.i
        self.__createMessageC("../..")
        if self.pathToExternalModule and os.path.exists(os.path.join(self.pathToExternalModule,"msgPayloadDefC")):
            self.__createMessageC(self.pathToExternalModule,True)

        with open(self.autoSourceDestDir + 'messaging.auto.i', 'r') as fb:
            self.messagingAutoData = fb.readlines()
        # The following cpp message definitions must be included after the `self.messagingAutoData` variable is set above.
        # We only need to create Python interfaces to C++ messages, not C wrappers.
        self.__createMessageCpp("../..")
        if self.pathToExternalModule and os.path.exists(
                os.path.join(self.pathToExternalModule, "msgPayloadDefCpp")):
            self.__createMessageC(self.pathToExternalModule, True)

    def __toMessage(self, structData):
        if structData:
            structData = structData.replace(' ', '').split(',')
            structName = structData[0]
            sourceHeaderFile = f"{structData[2]}/{structName}Payload.h"
            definitions = self.messageTemplate.format(type=structName)
            header = self.headerTemplate.format(type=structName, structHeader=sourceHeaderFile)
            self.swigTemplate.write(self.swig_template_block.format(type=structName))
            file_name = os.path.join(self.destinationDir, structName + '_C')
            definitionsFile = file_name + '.cpp'
            header_file = file_name + '.h'
            with open(definitionsFile, 'w') as w:
                w.write(definitions)
            with open(header_file, 'w') as w:
                w.write(header)
    def initialize(self):
        self.__createMessageAndHeaderTemplate()
        self.__recreateDestinationDirectory()
        self.__generateMessagingHeaderInterface()

    def run(self):

        self.__generateMessages()
        # create swig file for C-msg C interface methods
        self.swigTemplate = open(self.autoSourceDestDir + 'cMsgCInterfacePy.auto.i', 'w')
        templateCall = 'INSTANTIATE_TEMPLATES({:dat})'
        for line in self.messagingAutoData:
            parse.parse(templateCall, line.strip(), dict(dat=self.__toMessage))
        self.swigTemplate.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Configure generated Messages")
    # define the optional arguments
    parser.add_argument("--pathToExternalModule", help="External Module path", default="")
    args = parser.parse_args()
    generateMessages = GenerateMessages(args.pathToExternalModule)
    generateMessages.initialize()
    generateMessages.run()