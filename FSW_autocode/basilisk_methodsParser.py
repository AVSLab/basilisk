import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules/')
import AVSSim
import alg_contain
import sim_model
import numpy as np

# ----------------------------- METHODS ----------------------------- #
def parseSwigVars(list):
    parsed_array = np.array([])
    length = len(list)
    i = 0
    while i < length:
        if list[i][0]!='_':
            parsed_array = np.append(parsed_array, list[i])
            i += 1
        else:
            break
    return parsed_array

def evalParsedList(list):
    addressDict = {}
    for methodName in list:
        methodObject = eval('sys.modules["' + module + '"].' + methodName)
        objectSize = sys.getsizeof(methodObject)
        if objectSize == 48: # size of SwigPyObject
            methodAddress = sim_model.getObjectAddress(methodObject)
            addressDict[methodName] = int(methodAddress)
    print addressDict

# ----------------------------- MAIN ----------------------------- #
if __name__ == "__main__":
    TheAVSSim = AVSSim.AVSSim()
    taskIdxList = [10, 12, 13, 14, 15, 19, 20, 21, 22, 23, 24, 25, 26]
    #taskIdxList = [10]
    for i_task in taskIdxList:
        print 'Task Index = ', i_task
        task = TheAVSSim.TaskList[i_task]
        print 'Task Name = ', task.Name
        for model in task.TaskModels:
            module = model.__module__
            print 'Module = ', module
            sysMod = sys.modules[module]
            dirList = dir(sysMod)
            parsed_dirList = parseSwigVars(dirList)
            evalParsedList(parsed_dirList)
        print '\n'

    addressDictMatch = {}
    TheAVSList = dir(TheAVSSim)
    for elemName in TheAVSList:
        elem = eval('TheAVSSim.' + elemName)
        if type(elem) == alg_contain.AlgContain:
            addressDictMatch['SelfInit'] = int(elem.getSelfInitAddress())
            addressDictMatch['CrossInit'] = int(elem.getCrossInitAddress())
            addressDictMatch['Update'] = int(elem.getUpdateAddress())
            if (elem.getResetAddress()):
                addressDictMatch['Reset'] = int(elem.getResetAddress())
            print 'Wrap = ', elemName
            print 'Model Tag = ', elem.ModelTag
            print addressDictMatch
            print '\n'
        # elif sys.getsizeof(elem) == 64: # size of a class in the AVSSim
        #     print elemName
        # try:
        #     mod = elem.__module__
        #     print elemName
        #     print type(elem)
        # except:
        #     continue





