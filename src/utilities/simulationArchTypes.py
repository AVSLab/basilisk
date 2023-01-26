#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#



from Basilisk.architecture import sim_model
from Basilisk.architecture import sys_model_task


class ProcessBaseClass(object):
    """Class for a BSK process"""
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
        pass

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
    idCounter = 1
    def __init__(self, modelName, modelActive=True, modelPriority=-1):
        # The modelName is a unique identifier (unique to simulation) passed
        # in to a class.
        self.modelName = modelName
        # The modelActive flag indicates if the model should be run or not
        self.modelActive = modelActive
        # The moduleID is a numeric identifier used to track message usage in
        # a given simulation.
        # Note: python modules get negative ID numbers
        self.moduleID = -PythonModelClass.idCounter
        PythonModelClass.idCounter = PythonModelClass.idCounter + 1
        # The modelPriority variable is the setting for which models get run
        # first.  Higher priority indicates that a model will get run sooner.
        self.modelPriority = modelPriority

    # The selfInit method is used to initialize all of the output messages of a class.
    def selfInit(self):
        return

    # The reset method is used to clear out any persistent variables that need to get changed
    #  when a task is restarted.  This method is typically only called once after selfInit,
    #  but it should be written to allow the user to call it multiple times if necessary.
    def reset(self, currentTime):
        return

    # The updateState method is the cyclical worker method for a given Basilisk class.  It
    # will get called periodically at the rate specified in the Python task that the model is
    # attached to.  It persists and anything can be done inside of it.  If you have realtime
    # requirements though, be careful about how much processing you put into a Python updateState
    # method.  You could easily detonate your sim's ability to run in realtime.
    def updateState(self, currentTime):
        return


class PythonTaskClass(object):
    def __init__(self, taskName, taskRate, taskActive=True, taskPriority=-1, parentProc=None):
        self.name = taskName
        self.rate = taskRate
        self.priority = taskPriority
        self.modelList = []
        self.nextTaskTime = 0
        self.taskActive = taskActive
        self.parentProc = parentProc

    def updateParentProc(self, newParentProc):
        self.parentProc = newParentProc

    def selfInitTask(self):
        for model in self.modelList:
            model.selfInit()

    def resetTask(self, currentTime):
        for model in self.modelList:
            model.reset(currentTime)

    def addModelToTask(self, newModel, priority=None):
        if priority is not None:
            newModel.modelPriority = priority
        i = 0
        for model in self.modelList:
            if newModel.modelPriority > model.modelPriority:
                self.modelList.insert(i, newModel)
                return
            i += 1
        self.modelList.append(newModel)

    def executeModelList(self, currentTime):
        self.nextTaskTime = currentTime + self.rate
        if not self.taskActive:
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
                    newTask.nextTaskTime == tmpTask.nextTaskTime and
                    newTask.priority > tmpTask.priority):
                self.executionOrder.insert(i, newTask)
                return
        self.executionOrder.append(newTask)

    def createPythonTask(self, newTaskName, taskRate, taskActive=True, taskPriority=-1):
        self.taskList.append(PythonTaskClass(newTaskName, taskRate, taskActive, taskPriority, self.processData))

    def addPythonTask(self, newPyTask):
        self.taskList.append(newPyTask)

    def addModelToTask(self, taskName, newModel, priority=None):
        for task in self.taskList:
            if task.name == taskName:
                task.addModelToTask(newModel, priority)
                return
        print("Attempted to add model: " + newModel.modelName)
        print("to non-existent task: " + taskName)

    def selfInitProcess(self):

        if not self.taskList:
            return

        for task in self.taskList:
            task.selfInitTask()
        self.nextTaskTime = 0
        self.scheduleTask(self.taskList[-1])

    def resetProcess(self, currentTime):
        self.executionOrder = []
        for task in self.taskList:
            task.resetTask(currentTime)
            self.scheduleTask(task)

    def executeTaskList(self, currentTime):
        if len(self.executionOrder) == 0:
            return
        taskNext = self.executionOrder[0]
        for intCurr in self.intRefs:
            intCurr.routeInputs(self.processData.messageBuffer)
        while taskNext.nextTaskTime <= currentTime:
            taskNext.executeModelList(currentTime)
            self.executionOrder.pop(0)
            self.scheduleTask(taskNext)
            taskNext = self.executionOrder[0]
        self.nextTaskTime = taskNext.nextTaskTime

    def addInterfaceRef(self, newInt):
        self.intRefs.append(newInt)
