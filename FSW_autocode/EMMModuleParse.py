
'''This document contains technology controlled by the U.S. Export Administration
Regulations, 15 C.F.R. Parts 740-774 (EAR). Transfer, disclosure, or export to 
foreign persons without prior U.S. Government approval may be prohibited. 
Violations of these export laws and regulations are subject to severe civil 
and criminal penalties.
'''

import sys, os, inspect, re

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules')

# import AVSSim


# ----------------------------- METHODS ----------------------------- #

# This method simply makes a string of the format: 'self.TaskList[0].TaskModels[0]'
def makeTaskModelString(i,j):
    return 'self.TaskList['+str(i)+'].TaskModels['+str(j)+']'

# This method is the main autocode sequence. This method is NOT recursive.
def autocodeTaskLists():
    handledModels = []
    for i in TaskListIdxs: # loop through TaskLists
        auxFile.write('\n\n'+'TaskList: '+str(i)+'\n') # print current Tasklist index to workfile
        # print '\n\n'+'TaskList: '+str(i)+'\n'

        for j in range(0, len(TaskList[i].TaskModels)): # loop through TaskModels in TaskList
                auxFile.write('\n'+'TaskModel: '+str(j)+'\n') # print current taskmodel index to workfile
                # print '\n'+'TaskModel: '+str(j)+'\n'

                fieldStr = str(type(TaskList[i].TaskModels[j]).__name__)
                prefix = NameReplace[makeTaskModelString(i,j)]
                if(prefix in handledModels):
                    continue
                EMMInith.write('\t' + fieldStr + ' ' + prefix + ';\n')

                autocodeObject(TaskList[i].TaskModels[j],prefix)
                handledModels.append(prefix)

    EMMInitc.write('}')
    EMMInith.write('}EMMConfigData;')
    print("Goin' to Mars2")
    return

# This method recursively autocodes the input object.
def autocodeObject(input,prefix):
    fieldNames = dir(input) # list all the variable names under current TaskModel. input = TaskList[i].TaskModels[j]

    for k in range(0,len(fieldNames)): # loop through variables within current TaskModel

        fieldName = fieldNames[k]
        fieldValue = getattr(input,fieldName)
        fieldTypeName = type(fieldValue).__name__
        fieldTypeFull = str(type(fieldValue))

        if not (fieldName[0:2] == '__' or fieldName[0:4] == 'this'):
            # print fieldName
            # print fieldTypeFull
            # print fieldTypeName
            # print str(fieldValue)
            # print '\n'
            auxFile.write('\n'+fieldName+'\n') # print name of current element of taskmodel to workfile
            auxFile.write(fieldTypeFull+'\n') # print datatype to workfile
            auxFile.write(fieldTypeName+'\n') # print datatype to workfile
            auxFile.write(str(fieldValue)+'\n') # print value of current element to workfile

        # this and __ and SwigPyObject and instancemethod
        if (fieldName[0:2] == '__') or (fieldName[0:4] == 'this') or fieldTypeName == 'SwigPyObject' or fieldTypeName == 'instancemethod':
            continue # skip __ and this and SwigPyObject and instancemethod

        # class
        elif fieldTypeFull[1:6] == 'class':
            autocodeObject(fieldValue,prefix+'.'+fieldName)

        # list of class/struct
        elif fieldTypeName == 'list' and str(type(fieldValue[0]))[1:6] == 'class':
            for l in range(0,len(fieldValue)):
                autocodeObject(fieldValue[l],prefix+'.'+fieldName+'['+str(l)+']')

        # character array
        elif fieldTypeName == 'str':
            dest = ConfigDataStr + prefix + '.' + str(fieldName)
            EMMInitc.write('\t'+'strcpy(' + dest + ',' + '"'+str(fieldValue)+'"' + ')'+';\n')

        # handle numeric lists
        elif fieldTypeName == 'list':
            for l in range(0,len(fieldValue)):
                EMMInitc.write('\t' + ConfigDataStr + prefix + '.' + str(fieldName) + '[' + str(l) + '] = ' + str(fieldValue[l])+';\n')

        # non-array variable
        else:
            EMMInitc.write('\t' + ConfigDataStr + prefix + '.')
            EMMInitc.write(str(fieldName)) # name of the variable
            EMMInitc.write(' = '+str(fieldValue)+';\n') # value of the variable
    return


# ----------------------------- MAIN ----------------------------- #
if __name__ == "__main__":

    # TheAVSSim = AVSSim.AVSSim()

    # constants and such
    ConfigDataStr = 'ConfigData->' # name of the main struct
    arrMaxLen = 3
    NameReplace = TheAVSSim.NameReplace
    TaskList = TheAVSSim.TaskList
    TaskListIdxs = [2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 14, 15, 19, 20, 21, 22, 23, 24, 25, 26]

    # open the files for writing
    auxFile = open(path + '/../FSW_autocode/autocode_aux.txt', 'w') # auxFile is useful for debugging
    EMMInitc = open(path + '/../FSW_autocode/EMMInit.c', 'w') # .c file
    EMMInith = open(path + '/../FSW_autocode/EMMInit.h', 'w') # .h file

    # print initialization text to the files
    EMMInitc.write('void dataInitialization(EMMConfigData *ConfigData) {\n')
    EMMInith.write('typedef struct {\n')

    # autocode TheAVSSim!
    autocodeTaskLists()

    auxFile.close() # close auxFile
    EMMInitc.close() # close EMMInitc file
    EMMInith.close() # close EMMInith file
