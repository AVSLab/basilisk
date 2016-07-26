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
    for methodName in list:
        methodObject = eval('sys.modules["' + module + '"].' + methodName)
        objectSize = sys.getsizeof(methodObject)
        addressesArray = np.array([])
        if objectSize == 48: # size of SwigPyObject
            print methodName
            print sim_model.getObjectAddress(methodObject)

# ----------------------------- MAIN ----------------------------- #
if __name__ == "__main__":
    TheAVSSim = AVSSim.AVSSim()
    taskIdxList = [10, 12, 13, 14, 15, 19, 20, 21, 22, 23, 24, 25, 26]
    taskIdxList = [26]

    for i_task in taskIdxList:
        print 'Task Index = ', i_task
        task = TheAVSSim.TaskList[i_task]
        print 'Task Name = ', task.Name
        print '\n'
        for model in task.TaskModels:
            module = model.__module__
            print 'Module = ', module
            sysMod = sys.modules[module]
            dirList = dir(sysMod)
            parsed_dirList = parseSwigVars(dirList)
            evalParsedList(parsed_dirList)
            print '\n'
        print '\n'

    TheAVSList = dir(TheAVSSim)
    for elemName in TheAVSList:
        elem = eval('TheAVSSim.' + elemName)
        if type(elem) == alg_contain.alg_contain.AlgContain:
            print 'YAY'
            print elemName
            print elem.getSelfInitAddress()
            print dir(elem)
            print '\n'




