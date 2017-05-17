import sim_model
import sys_model_task

class ProcessBaseClass(object):
    def __init__(self, procName):
        self.Name = procName
        self.processData = sim_model.SysProcess(procName)

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
    def __init__(self, modelName, modelActive = True, modelPriority = -1):
        self.modelName = modelName
        self.modelActive = modelActive
        self.modelPriority = -1
    def selfInit(self):
        print "Uhhh the model: " + self.modelName + " is just the python base class"
        return
    def crossInit(self):
        return
    def reset(self, currentTime):
        return
    def updateState(self, currentTime):
        return

class PythonTaskClass(object):
    def __init__(self, taskName, taskRate, taskActive = True, taskPriority=-1):
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
        i=0
        for model in self.modelList:
            if(newModel.modelPriority > model.modelPriority):
                self.modelList.insert(i, newModel)
                return
            i+=1
        self.modelList.append(newModel) 
            
    def executeModelList(self, currentTime):
        self.nextTaskTime = currentTime + self.rate
        if(self.taskActive != True):
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
            if(newTask.nextTaskStart < tmpTask.nextTaskStart or \
                newTask.nextTaskStart == tmpTask.nextTaskStart and \
                newTask.priority > tmpTask.priority):
                self.executionOrder.insert(i, newTask)
                return
        self.executionOrder.append(newTask)
         
    def createPythonTask(self, newTaskName, taskRate, taskActive = True, taskPriority=-1):
        self.taskList.append(PythonTaskClass(newTaskName, taskRate, taskActive, taskPriority))
    def addPythonTask(self, newPyTask):
        self.taskList.append(newPyTask) 
    def addModelToTask(self, taskName, newModel, priority=None):
        for task in self.taskList:
            if(task.name == taskName):
                task.addModelToTask(newModel, priority)
                return
        print "Attempted to add model: " + newModel.modelName
        print "to non-existent task: " + taskName
    def selfInitProcess(self):
        self.processData.selectProcess()
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
        taskNext = self.executionOrder[0]
        for intCurr in self.intRefs:
            intCurr.routeInputs(self.processData.messageBuffer)
        self.processData.selectProcess()
        while(taskNext.nextTaskTime <= currentTime):
            taskNext.executeModelList(currentTime)
            self.executionOrder.pop(0)
            self.scheduleTask(taskNext)
            taskNext = self.executionOrder[0]
        self.nextTaskTime = taskNext.nextTaskTime
    def addInterfaceRef(self, newInt):
        self.intRefs.append(newInt)

