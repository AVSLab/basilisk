from mpy_cpp_wrapper import CppWrapperClass
import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../fsw_examples/')
from desktopFSW_sim import DesktopFSW


def empty_wraps_folder(outputPath):
    """
    This function cleans the directory where output will be stored provided that such directory exists.
    If it doesn't exist, it will create a fresh one.
    :param outputPath: absolute folder plath where the outputs will be stored
    :return:
    """
    if os.path.exists(outputPath):
        print("Cleaning directory: %s" % outputPath)
        for plot_file in os.listdir(outputPath):
            os.remove(outputPath + '/' + plot_file)
    else:
        print("Creating new directory: %s" % outputPath)
        os.makedirs(outputPath)


def run_auto_wrapper():
    """
       This is the main function to run for creating C++ wrapper classes around the existing C modules.
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
    outputPath = os.path.dirname(os.path.abspath(filename)) + "/wraps"
    empty_wraps_folder(outputPath=outputPath)

    # Run the auto-wrapper
    CppWrapperClass(TheSim=TheSim, taskActivityDir=taskActivityDir,
                    simTag="TheSim.fswModels", outputPath=outputPath)


if __name__ == "__main__":
    run_auto_wrapper()
