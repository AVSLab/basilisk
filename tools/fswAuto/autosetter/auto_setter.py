import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../fsw_examples/')
from desktopFSW_sim import DesktopFSW

import methodsParser


def empty_sets_folder(outputPath):
    """
    This function cleans the directory where output will be stored provided that such directory exists.
    If it doesn't exist, it will create a fresh one.
    :param outputPath: absolute folder plath where the outputs will be stored
    :return:
    """
    if os.path.exists(outputPath):
        print("Cleaning directory: %s" % outputPath)
        for data_file in os.listdir(outputPath):
            os.remove(outputPath + '/' + data_file)
    else:
        print("Creating new directory: %s" % outputPath)
        os.makedirs(outputPath)


def run_auto_setter():
    """
       This is the main function to translate Python initialization/setup code into C.
       Detailed comments are provided below.
    """
    # Import your FSW simulation
    TheSim = DesktopFSW()

    # Define the FSW tasks whose modules are to be parsed
    taskActivityDir = dict()
    taskActivityDir["initOnlyTask"] = str(0)
    taskActivityDir["inertial3DPointTask"] = str(0)
    taskActivityDir["feedbackControlTask"] = str(0)

    # Define the path where the C++ wrapper classes will be stored
    output_path = os.path.dirname(os.path.abspath(filename)) + "/sets"
    empty_sets_folder(outputPath=output_path)

    # Pick names for your output C data
    outputFileName = 'FSW_autoset'
    str_ConfigData = 'config_data'

    # Run the auto-setter
    methodsParser.parseSimAlgorithms(TheSim=TheSim, taskActivityDir=taskActivityDir,
                                     outputCFileName=outputFileName,
                                     str_ConfigData=str_ConfigData,
                                     simTag="TheSim.fswModels",
                                     localPath=output_path)


if __name__ == "__main__":
    run_auto_setter()
