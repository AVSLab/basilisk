# Migrating BSK Scripts to Python 3 {#migratingToPython3}

With release Basilisk v0.8.x onward the software framework now supports using Python 3.  The purpose of this document is to illustrate how to migrate Python 2 BSK scripts such that they will function in both Python 3 and Python 2.  For the time being Python 2 is still supported as a depreciated functionality.  But, python scripts committed to Basilisk should be written such that they support Python 3 and 2 for now. 

## Importing Python Modules
With Python 2 BSK scripts it was possible to use an indirect method of importing Python modules.  For example, consider this old import method
```
from Basilisk.fswAlgorithms import fswModuleTemplate
```
This must be upgraded to use a direct import method such as:
```
from Basilisk.fswAlgorithms.fswModuleTemplate import fswModuleTemplate
```


## Dividing Scalars
Python 2 and 3 treat the devide operator `/` differently if two integers operated on.  Thus
```
    a = 3/2
```
resulted in an integer value of 1 in Python 2, but yields a float value of 1.5 in Python 3.  To get the same result in Python 3 and 2, you can use either of the following options which work in both version of Python:
```
    a = 3//2       
    a = 3./2 
```
Without modification the user will see an error in Python 3 complaining about an unsupported type conversion.
```
File "/Users/hp/Documents/Research/Basilisk/dist3/Basilisk/simulation/sim_model/sim_model.py", line 4351, in logThisMessage
    return _sim_model.SimModel_logThisMessage(self, messageName, messagePeriod)
NotImplementedError: Wrong number or type of arguments for overloaded function 'SimModel_logThisMessage'.
  Possible C/C++ prototypes are:
    SimModel::logThisMessage(std::string,uint64_t)
    SimModel::logThisMessage(std::string)
```


