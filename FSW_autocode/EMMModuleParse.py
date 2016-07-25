
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
import AVSSim

TheAVSSim = AVSSim.AVSSim()

f = open(path + '/../../Basilisk/FSW_autocode/workfile.txt', 'w')

EMMInitc = open(path + '/../../Basilisk/FSW_autocode/EMMInit.c', 'w')
EMMInith = open(path + '/../../Basilisk/FSW_autocode/EMMInit.h', 'w')


EMMInitc.write('void dataInitialization(EMMConfigData *ConfigData) {\n')
EMMInith.write('typedef struct {\n')

ConfigDatastr = 'ConfigData->'

nameReplaceDict = TheAVSSim.NameReplace

def matchDictValueToKey(searchVal):
    return nameReplaceDict.keys()[nameReplaceDict.values().index(searchVal)]

taskListIn = TheAVSSim.TaskList
nTaskListIn = len(taskListIn)

for i in range(10, 11): # only process TaskList[2] for now
    print('\n\n'+'TaskList: '+str(i)+'\n') # print current tasklist to terminal
    f.write('\n\n'+'TaskList: '+str(i)+'\n') # print current tasklist to workfile
    nTaskModels = len(taskListIn[i].TaskModels)

    for j in range(0, nTaskModels): # loop through TaskModels in TaskList
        print('TaskModel: '+str(j)+'\n') # print current taskmodel to terminal
        f.write('TaskModel: '+str(j)+'\n') # print current taskmofrl to workfile
        curElems = dir(taskListIn[i].TaskModels[j]) # list all the elements under current TaskModel
        nCurElems = len(curElems)
        fieldStr = str(type(taskListIn[i].TaskModels[j]).__name__)
        fieldStrInterface = matchDictValueToKey('self.TaskList['+str(i)+'].TaskModels['+str(j)+']')
        EMMInith.write('\t' + fieldStr + ' ' + fieldStrInterface + ';\n')

        for k in range(0, nCurElems): # loop through elements within current TaskModel
            print(curElems[k]) # print name of current element of taskmodel to terminal
            f.write('\n'+str(curElems[k])+'\n') # print name of current element of taskmodel to workfile

            if (curElems[k][0:2] != '__') & (curElems[k][0:4] != 'this'): # make sure element doesn't begin with __ or this
                curAttr = getattr(taskListIn[i].TaskModels[j], curElems[k]) # get value of current element of TaskModel
                print(type(curAttr)) # print datatype of value of current element to terminal
                f.write(str(type(curAttr))+'\n') # print datatype to workfile

                if (str(type(curAttr))[1:6] != 'class'): # skip classes
                    print(curAttr) # print value of current element to terminal
                    print curAttr.__sizeof__()
                    f.write(str(curAttr.__sizeof__())+'\n')
                    f.write(str(curAttr)+'\n') # print value of current element to workfile
                    curTypeStr = type(curAttr).__name__ # datatype of value of current element

                    if curTypeStr == 'str': # write sequence for string datatype
                        dest = ConfigDatastr + fieldStrInterface + '.' + str(curElems[k])
                        src = '"'+str(curAttr)+'"'
                        strcpyStr = 'strcpy(' + dest + ',' + src + ')'
                        EMMInitc.write('\t'+strcpyStr+';\n')
                    elif curTypeStr == 'SwigPyObject':
                        lmax = 10
                        typestr = curAttr.__str__()
                        strindx = typestr.find("'")
                        typestr = typestr[(strindx+1):len(typestr)]
                        strindx = typestr.find("'")
                        typestr = typestr[0:strindx]
                        strindx = typestr.find(' *')

                        if strindx >= 0:
                            typestr = typestr[0:strindx]

                        if typestr.find('int') >= 0:
                            typestr = typestr[typestr.find('int'):(typestr.find('int')+3)]

                        print typestr
                        print curAttr
                        arr = AVSSim.SimulationBaseClass.getCArray(typestr,curAttr,lmax)

                        for l in range(0,lmax):
                            EMMInitc.write('\t' + ConfigDatastr + fieldStrInterface + '.' + str(curElems[k]) + '[' + str(l) + '] = ' + str(arr[l])+';\n')
                    else:
                        EMMInitc.write('\t' + ConfigDatastr + fieldStrInterface + '.')
                        EMMInitc.write(str(curElems[k])) # name of the variable
                        EMMInitc.write(' = '+str(curAttr)+';\n') # value of the variable

EMMInitc.write('}')
EMMInith.write('}EMMConfigData;')

f.close() # close workfile
EMMInitc.close() # close c_out file
EMMInith.close() # close c_out file
print('done')
