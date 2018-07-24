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



from Basilisk.simulation import sim_model
from Basilisk.simulation import sys_model_task


def CreateNewMessage(messageName, messageType, moduleID):
    messageStructName = messageType.__class__.__name__
    messageID = sim_model.SystemMessaging_GetInstance().CreateNewMessage(
        messageName, messageType.getStructSize(), 2, messageStructName, moduleID)
    return messageID


def SubscribeToMessage(messageName, messageType, moduleID):
    messageID = sim_model.SystemMessaging_GetInstance().subscribeToMessage(
        messageName, messageType.getStructSize(), moduleID)
    return messageID


def ReadMessage(messageID, messageType, moduleID):
    localHeader = sim_model.SingleMessageHeader()
    sim_model.SystemMessaging_GetInstance().ReadMessage(
        messageID, localHeader, messageType.getStructSize(), messageType, moduleID)
    return localHeader


def WriteMessage(messageID, currentTime, messageStruct, moduleID, msgSize=-1):
    if msgSize <= 0:
        msgSize = messageStruct.getStructSize()
    sim_model.SystemMessaging_GetInstance().WriteMessage(
        messageID, currentTime, msgSize, messageStruct, moduleID)
    return


class ProcessBaseClass(object):
    def __init__(self, procName, procPriority=-1):
        self.Name = procName
        self.processData = sim_model.SysProcess(procName)
        self.processData.processPriority = procPriority

    def addTask(self, newTask, taskPriority=-1):
        self.processData.addNewTask(newTask.TaskData, taskPriority)

    def addInterfaceRef(self, newInt):
        self.processData.addInterfaceRef(newInt)

    def discoverAllMessages(self):
        self.processData.discoverAllMessages()

    def disableAllTasks(self):
        self.processData.disableAllTasks()

    def enableAllTasks(self):
        self.processData.enableAllTasks()

    def selectProcess(self):
        self.processData.selectProcess()

    def updateTaskPeriod(self, TaskName, newPeriod):
        self.processData.changeTaskPeriod(TaskName, newPeriod)


class TaskBaseClass(object):
    def __init__(self, TaskName, TaskRate, InputDelay=0, FirstStart=0):
        self.Name = TaskName
        self.TaskData = sys_model_task.SysModelTask(TaskRate, InputDelay,
                                                    FirstStart)
        self.TaskData.TaskName = TaskName
        self.TaskModels = []

    def disable(self):
        self.TaskData.disableTask()

    def enable(self):
        self.TaskData.enableTask()

    def resetTask(self, callTime):
        self.TaskData.ResetTaskList(callTime)


class PythonModelClass(object):
    def __init__(self, modelName, modelActive=True, modelPriority=-1):
        ## The modelName is a unique identifier (unique to simulation) passed
        # in to a class.
        self.modelName = modelName
        ## The modelActive flag indicates if the model should be run or not
        self.modelActive = modelActive
        ## The moduleID is a numeric identifier used to track message usage in
        # a given simulation.
        self.moduleID = sim_model.SystemMessaging_GetInstance().checkoutModuleID()
        ## The modelPriority variable is the setting for which models get run
        # first.  Higher priority indicates that a model will get run sooner.
        self.modelPriority = modelPriority

    ## The selfInit method is used to initialze all of the output messages of a class.
    # It is important that ALL outputs are initialized here so that other models can
    # subscribe to these messages in their crossInit method.
    def selfInit(self):
        print "Uhhh the model: " + self.modelName + " is just the python base class"
        return

    ## The crossInit method is used to initialize all of the input messages of a class.
    #  This subscription assumes that all of the other models present in a given simulation
    #  instance have initialized their messages during the selfInit step.
    def crossInit(self):
        return

    ## The reset method is used to clear out any persistent variables that need to get changed
    #  when a task is restarted.  This method is typically only called once after selfInit/crossInit,
    #  but it should be written to allow the user to call it multiple times if necessary.
    def reset(self, currentTime):
        return

    ## The updateState method is the cyclical worker method for a given Basilisk class.  It
    # will get called periodically at the rate specified in the Python task that the model is
    # attached to.  It persists and anything can be done inside of it.  If you have realtime
    # requirements though, be careful about how much processing you put into a Python updateState
    # method.  You could easily detonate your sim's ability to run in realtime.
    def updateState(self, currentTime):
        return


class PythonTaskClass(object):
    def __init__(self, taskName, taskRate, taskActive=True, taskPriority=-1):
        self.name = taskName
        self.rate = taskRate
        self.priority = taskPriority
        self.modelList = []
        self.nextTaskTime = 0
        self.taskActive = taskActive

    def selfInitTask(self):
        for model in self.modelList:
            model.selfInit()

    def crossInitTask(self):
        for model in self.modelList:
            model.crossInit()

    def resetTask(self, currentTime):
        for model in self.modelList:
            model.reset(currentTime)

    def addModelToTask(self, newModel, priority=None):
        if priority is not None:
            newModel.modelPriority = priority
        i = 0
        for model in self.modelList:
            if (newModel.modelPriority > model.modelPriority):
                self.modelList.insert(i, newModel)
                return
            i += 1
        self.modelList.append(newModel)

    def executeModelList(self, currentTime):
        self.nextTaskTime = currentTime + self.rate
        if (self.taskActive != True):
            return
        for model in self.modelList:
            model.updateState(currentTime)


class PythonProcessClass(ProcessBaseClass):
    def __init__(self, procName, priority=-1):
        super(PythonProcessClass, self).__init__(procName)
        self.taskList = []
        self.executionOrder = []
        self.nextTaskTime = 0
        self.pyProcPriority = priority
        self.intRefs = []

    def nextCallTime(self):
        return self.nextTaskTime

    def scheduleTask(self, newTask):
        for i in range(len(self.executionOrder)):
            tmpTask = self.executionOrder[i]
            if (newTask.nextTaskTime < tmpTask.nextTaskTime or
                    newTask.nextTaskTime == tmpTask.nextTaskTime and \
                    newTask.priority > tmpTask.priority):
                self.executionOrder.insert(i, newTask)
                return
        self.executionOrder.append(newTask)

    def createPythonTask(self, newTaskName, taskRate, taskActive=True, taskPriority=-1):
        self.taskList.append(PythonTaskClass(newTaskName, taskRate, taskActive, taskPriority))

    def addPythonTask(self, newPyTask):
        self.taskList.append(newPyTask)

    def addModelToTask(self, taskName, newModel, priority=None):
        for task in self.taskList:
            if (task.name == taskName):
                task.addModelToTask(newModel, priority)
                return
        print "Attempted to add model: " + newModel.modelName
        print "to non-existent task: " + taskName

    def selfInitProcess(self):
        self.processData.selectProcess()

        if not self.taskList:
            return

        for task in self.taskList:
            task.selfInitTask()
        self.nextTaskTime = 0
        self.scheduleTask(self.taskList[-1])

    def crossInitProcess(self):
        self.processData.selectProcess()
        for task in self.taskList:
            task.crossInitTask()

    def resetProcess(self, currentTime):
        self.executionOrder = []
        self.processData.selectProcess()
        for task in self.taskList:
            task.resetTask(currentTime)
            self.scheduleTask(task)

    def executeTaskList(self, currentTime):
        if (len(self.executionOrder) == 0):
            return
        taskNext = self.executionOrder[0]
        for intCurr in self.intRefs:
            intCurr.routeInputs(self.processData.messageBuffer)
        self.processData.selectProcess()
        while (taskNext.nextTaskTime <= currentTime):
            taskNext.executeModelList(currentTime)
            self.executionOrder.pop(0)
            self.scheduleTask(taskNext)
            taskNext = self.executionOrder[0]
        self.nextTaskTime = taskNext.nextTaskTime

    def addInterfaceRef(self, newInt):
        self.intRefs.append(newInt)
