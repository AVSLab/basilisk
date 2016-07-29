
'''This document contains technology controlled by the U.S. Export Administration
Regulations, 15 C.F.R. Parts 740-774 (EAR). Transfer, disclosure, or export to 
foreign persons without prior U.S. Government approval may be prohibited. 
Violations of these export laws and regulations are subject to severe civil 
and criminal penalties.
'''

import sys, os, inspect, re

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../../Basilisk/PythonModules')

# import AVSSim
# TheAVSSim = AVSSim.AVSSim()

# open the files for writing
wf = open(path + '/../../Basilisk/FSW_autocode/workfile.txt', 'w') # workfile is useful for debugging
EMMInitc = open(path + '/../../Basilisk/FSW_autocode/EMMInit.c', 'w') # .c file
EMMInith = open(path + '/../../Basilisk/FSW_autocode/EMMInit.h', 'w') # .h file

# print initialization text to the files
EMMInitc.write('void dataInitialization(EMMConfigData *ConfigData) {\n')
EMMInith.write('typedef struct {\n')

# name of the main struct
ConfigDataStr = 'ConfigData->'


nameReplaceDict = TheAVSSim.NameReplace

def matchDictValueToKey(searchVal):
    return nameReplaceDict.keys()[nameReplaceDict.values().index(searchVal)]

arrMaxLen = 3

TaskList = TheAVSSim.TaskList

taskIdxList = [10, 12, 13, 14, 15, 19, 20, 22, 23, 24, 25, 21, 26] #

# make sure all TaskModels have a NameReplace
for i in range(0,len(TaskList)):
    for j in range(0,len(TaskList[i].TaskModels)):
        if not 'self.TaskList['+str(i)+'].TaskModels['+str(j)+']' in nameReplaceDict.itervalues():
            nameReplaceDict['TaskList_'+str(i)+'_TaskModel_'+str(j)] = 'self.TaskList['+str(i)+'].TaskModels['+str(j)+']'



for i in taskIdxList: # only process TaskList[2] for now

    # print info to terminal and workfile
    print('\n\n'+'TaskList: '+str(i)+'\n') # print current tasklist index to terminal
    wf.write('\n\n'+'TaskList: '+str(i)+'\n') # print current tasklist index to workfile

    for j in range(0, len(TaskList[i].TaskModels)): # loop through TaskModels in TaskList

            # print info to terminal and workfile
            print('TaskModel: '+str(j)+'\n') # print current taskmodel index to terminal
            wf.write('TaskModel: '+str(j)+'\n') # print current taskmodel index to workfile

            fieldStr = str(type(TaskList[i].TaskModels[j]).__name__)
            fieldStrInterface = matchDictValueToKey('self.TaskList['+str(i)+'].TaskModels['+str(j)+']')
            EMMInith.write('\t' + fieldStr + ' ' + fieldStrInterface + ';\n')

            varNameList = dir(TaskList[i].TaskModels[j]) # list all the variable names under current TaskModel

            for k in range(0, len(varNameList)): # loop through variables within current TaskModel
                wf.write('\n'+str(varNameList[k])+'\n') # print name of current element of taskmodel to workfile

                if (varNameList[k][0:2] != '__') & (varNameList[k][0:4] != 'this'): # make sure element doesn't begin with __ or this
                    varValue = getattr(TaskList[i].TaskModels[j], varNameList[k]) # get value of current element of TaskModel
                    wf.write(str(type(varValue))+'\n') # print datatype to workfile

                    if (str(type(varValue))[1:6] != 'class'): # skip classes
                        wf.write(str(varValue)+'\n') # print value of current element to workfile
                        varType = type(varValue).__name__ # datatype of value of current element

                        if varType == 'str': # write sequence for string datatype
                            dest = ConfigDataStr + fieldStrInterface + '.' + str(varNameList[k])
                            src = '"'+str(varValue)+'"'
                            strcpyStr = 'strcpy(' + dest + ',' + src + ')'
                            EMMInitc.write('\t'+strcpyStr+';\n')
                        elif varType == 'SwigPyObject':
                            typestr = varValue.__str__()
                            if typestr.find("void") >= 0 or typestr.find("AlgContain") >= 0:
                                continue
                            else:
                                strindx = typestr.find("'")
                                typestr = typestr[(strindx+1):len(typestr)]
                                strindx = typestr.find("'")
                                typestr = typestr[0:strindx]
                                strindx = typestr.find(' *')


                            if strindx >= 0:
                                typestr = typestr[0:strindx]


                            if typestr.find('int') >= 0:
                                typestr = typestr[typestr.find('int'):(typestr.find('int')+3)]

                            print varNameList[k]
                            print typestr
                            print varValue
                            print arrMaxLen
                            arr = AVSSim.SimulationBaseClass.getCArray(typestr,varValue,arrMaxLen)
                            # arr = [0]*10

                            for l in range(0,arrMaxLen):
                                EMMInitc.write('\t' + ConfigDataStr + fieldStrInterface + '.' + str(varNameList[k]) + '[' + str(l) + '] = ' + str(arr[l])+';\n')
                        else:
                            EMMInitc.write('\t' + ConfigDataStr + fieldStrInterface + '.')
                            EMMInitc.write(str(varNameList[k])) # name of the variable
                            EMMInitc.write(' = '+str(varValue)+';\n') # value of the variable

EMMInitc.write('}')
EMMInith.write('}EMMConfigData;')

wf.close() # close workfile
EMMInitc.close() # close c_out file
EMMInith.close() # close c_out file

print('done')
