''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''
def ParseConfigData(TheSim, TaskListIdxs, dataStr, sourceFile):
    # This method simply makes a string of the format: 'self.TaskList[0].TaskModels[0]'
    def makeTaskModelString(i,j):
        return 'self.TaskList['+str(i)+'].TaskModels['+str(j)+']'
    # This method is the main autocode sequence. This method is NOT recursive.
    def autocodeTaskLists():
        handledModels = []
        for i in TaskListIdxs: # loop through TaskLists
            for j in range(0, len(TaskList[i].TaskModels)):
                prefix = NameReplace[makeTaskModelString(i,j)]
                if(prefix in handledModels):
                    continue
                autocodeObject(TaskList[i].TaskModels[j],prefix, sourceFile)
                handledModels.append(prefix)

    def printList(input, prefix, sourceFile):
        for i in range(len(input)):
            prefixIn = prefix + '[' + str(i) + ']'
            if type(input[i]).__name__ == 'list':
                printList(input[i], prefixIn, sourceFile)
            else:
                sourceFile.write(prefixIn + ' = ' + str(input[i])+';\n')
            
    # This method recursively autocodes the input object.
    def autocodeObject(input, prefix, sourceFile):
        fieldNames = dir(input) # list all the variable names under current TaskModel. input = TaskList[i].TaskModels[j]
        for k in range(0,len(fieldNames)): # loop through variables within current TaskModel
            fieldName = fieldNames[k]
            fieldValue = getattr(input,fieldName)
            fieldTypeName = type(fieldValue).__name__
            fieldTypeFull = str(type(fieldValue))

            # this and __ and SwigPyObject and instancemethod
            if (fieldName[0:2] == '__' or fieldName[0:4] == 'this'
                or fieldTypeName == 'SwigPyObject' or fieldTypeName == 'instancemethod'):
                continue # skip
            # class
            elif fieldTypeFull[1:6] == 'class':
                autocodeObject(fieldValue, prefix + '.' + fieldName, sourceFile)
            # list of class/struct
            elif fieldTypeName == 'list' and (str(type(fieldValue[0]))[1:6] == 'class'):
                for l in range(0,len(fieldValue)):
                    autocodeObject(fieldValue[l], prefix + '.' + fieldName + '[' + str(l) + ']', sourceFile)
            # character array
            elif fieldTypeName == 'str':
                dest = dataStr + prefix + '.' + str(fieldName)
                sourceFile.write('\t' + 'strcpy(' + dest + ',' + '"' + str(fieldValue) + '"' + ')' + ';\n')
            # handle numeric lists
            elif fieldTypeName == 'list':
                printList(fieldValue, '\t' + dataStr + prefix + '.' + str(fieldName), sourceFile)
            # non-array variable
            else:
                sourceFile.write('\t' + dataStr + prefix + '.')
                sourceFile.write(str(fieldName)) # name of the variable
                sourceFile.write(' = '+str(fieldValue)+';\n') # value of the variable

    NameReplace = TheSim.NameReplace
    TaskList = TheSim.TaskList
    autocodeTaskLists()
