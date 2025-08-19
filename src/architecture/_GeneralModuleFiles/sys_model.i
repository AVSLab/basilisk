
%module cSysModel

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
   #include "sys_model.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_conly_data.i"
%include "swig_std_array.i"
%include "architecture/utilities/bskLogging.h"

%include "sys_model.h"

%pythonbegin %{
from typing import Union, Iterable
%}

%extend SysModel
{
    %pythoncode %{
        def logger(self, variableNames: Union[str, Iterable[str]], recordingTime: int = 0):
            """Generate a logger from one or more variables in this model.

            Each variable must be public or private with a standard-named getter.
            For example, if you want to log the variable `foo` from the model
            `mod`, then `mod.foo` or `mod.getFoo()` must be available.

            Args:
                variableNames (Union[str, Iterable[str]]): The name or names
                    of the variables to log.
                recordingTime (int, optional): The minimum interval between variable
                    recordings. Defaults to 0.
            """
            if isinstance(variableNames, str):
                variableNames = [variableNames]

            loggingFunctions = {}
            for variableName in variableNames:
                if hasattr(self, variableName):
                    loggingFunctions[variableName] = lambda _, variableName=variableName: getattr(self, variableName)
                    continue

                getterStr = f"get{variableName[0].upper()}{variableName[1:]}"
                getter = getattr(self, getterStr, None)
                if getter is not None:
                    loggingFunctions[variableName] = lambda _, getter=getter: getter()
                    continue

                raise ValueError(f"Cannot log {variableName} as it is not a "
                    f"public variable of {type(self).__name__} and the getter "
                    f"{getterStr} does not exist.")

            from Basilisk.utilities import pythonVariableLogger
            return pythonVariableLogger.PythonVariableLogger(loggingFunctions, recordingTime)
    %}
}
