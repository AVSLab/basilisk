import multiprocessing as mp
import os
import pickle

import numpy as np
import pandas as pd


class DataWriter(mp.Process):
    """ Class to be launched as separate process to pull data from queue and write out to .csv dataFrames
        Args:
            q: queue object from multiprocessing.Manager.queue
        Returns:
            Nil
    """
    def __init__(self, q):
        super(DataWriter, self).__init__()
        self._queue = q
        self._endToken = None
        self._varCast = None
        self._logDir = ""
        self._dataFiles = set()

    def run(self):
        """ The process run loop. Gets data from a queue and writes it out to per message csv files
            Args:
                Nil
            Returns:
                Nil
        """
        while self._endToken is None:
            data, mcSimIndex, self._endToken = self._queue.get()
            if self._endToken:
                continue
            for dictName, dictData in data.items(): # Loops through Messages, Variables, Custom dictionaries in the retention policy
                for itemName, itemData in dictData.items(): # Loop through all items and their data

                    if itemName == "OrbitalElements.Omega": # Protects from OS that aren't case sensitive.
                        itemName = "OrbitalElements.Omega_Capital"

                    filePath = self._logDir + itemName + ".data"
                    self._dataFiles.add(filePath)

                    # Is the data a vector, scalar, or non-existant?
                    try:
                        variLen = itemData[:,1:].shape[1]
                    except:
                        variLen = 0

                    # Generate the MultiLabel
                    outerLabel = [mcSimIndex]
                    innerLabel = []

                    for i in range(variLen):
                        innerLabel.append(i)
                    if variLen == 0:
                        innerLabel.append(0) # May not be necessary, might be able to leave blank and get a None
                    labels = pd.MultiIndex.from_product([outerLabel, innerLabel], names=["runNum", "varIdx"])

                    # Generate the individual run's dataframe
                    if variLen >= 2:
                        df = pd.DataFrame(itemData[:, 1:].tolist(), index=itemData[:,0], columns=labels)
                    elif variLen == 1:
                        df = pd.DataFrame(itemData[:, 1].tolist(), index=itemData[:,0], columns=labels)
                    else:
                        df = pd.DataFrame([np.nan], columns=labels)

                    for i in range(0, variLen):
                        try: # if the data is numeric reduce it to float32 rather than float64 to reduce storage footprint
                            # Note: You might think you can simplify these three lines into a single:
                            # df.iloc[:,i] = df.iloc[:,i].apply(pandas.to_numeric, downcast="float")
                            # but you'd be wrong.
                            varComp = df.iloc[:,i]
                            if self._varCast != None:
                                varComp = pd.to_numeric(varComp, downcast='float')
                            df.iloc[:,i] = varComp
                        except:
                            pass

                    # If the .data file doesn't exist save the dataframe to create the file
                    # and skip the remainder of the loop
                    if not os.path.exists(filePath):
                        pickle.dump([df], open(filePath, "wb"))
                        continue

                    # If the .data file does exists, append the message's pickle.
                    with open(filePath, "a+b") as pkl:
                        pickle.dump([df], pkl)

        # Sort by the MultiIndex (first by run number then by variable component)
        for filePath in self._dataFiles:
            # We create a new index so that we populate any missing run data (in the case that a run breaks) with NaNs.
            allData = []
            with open(filePath, 'rb') as pkl:
                try:
                    while True:
                        allData.extend(pickle.load(pkl))
                except EOFError:
                    pass
            allData = pd.concat(allData, axis=1)
            newMultInd = pd.MultiIndex.from_product([list(range(allData.columns.min()[0], allData.columns.max()[0]+1)),
                                                         list(range(allData.columns.min()[1], allData.columns.max()[1]+1))],
                                                         names=["runNum", "varIdx"])
            #allData = allData.sort_index(axis=1, level=[0,1]) #TODO: When we dont lose MCs anymore, we should just use this call
            allData = allData.reindex(columns=newMultInd)
            allData.index.name = 'time[ns]'
            allData.to_pickle(filePath)

    def setLogDir(self, logDir):
        self._logDir = logDir

    def setVarCast(self, varCast):
        self._varCast = varCast
