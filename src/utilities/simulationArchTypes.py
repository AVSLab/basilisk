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
    def __init__(self, TaskName, TaskRate, FirstStart=0):
        self.Name = TaskName
        self.TaskData = sys_model_task.SysModelTask(TaskRate, FirstStart)
        self.TaskData.TaskName = TaskName
        self.TaskModels = []

    def disable(self):
        self.TaskData.disableTask()

    def enable(self):
        self.TaskData.enableTask()

    def resetTask(self, callTime):
        self.TaskData.ResetTaskList(callTime)
