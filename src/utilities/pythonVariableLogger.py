import keyword
from typing import Callable, Any, Dict

import numpy as np

from Basilisk.architecture import sysModel

LoggingFunction = Callable[[int], Any]

class PythonVariableLogger(sysModel.SysModel):
    """This Python module calls one or multiple functions
    and stores their results. After the simulation is over, these
    values can be retrieved.

    This object is built with a dictionary of functions, and the
    results can be retrieved from the object using the keys of
    this dictionary::

        log = PythonVariableLogger({
            "a": lambda CurrentSimNanos: CurrentSimNanos**2,
            "b": lambda CurrentSimNanos: CurrentSimNanos**3,
        })

        # Run simulation

        times        = log.times()
        timesSquared = log.a
        timesCubed   = log.b
    """
    def __new__(cls, logging_functions: Dict[str, LoggingFunction], min_log_period: int = 0):
        # Generate an instance specific class that has properties for each logging function
        # NOTE: properties must be set on the class, not the instance itself
        properties = {
            name: property(
                    lambda self, name=name: np.array(self._variables[name])
                )
                for name in logging_functions
                if name.isidentifier() and not keyword.iskeyword(name) and not hasattr(cls, name)
        }
        dyncls = type('PythonVariableLoggerDynamic', (cls, ), properties)

        return super(PythonVariableLogger, dyncls).__new__(dyncls)

    def __init__(
        self,
        logging_functions: Dict[str, LoggingFunction],
        min_log_period: int = 0
    ) -> None:
        """Initializer.

        Args:
            logging_functions (Dict[str, LoggingFunction]): A dictionary where the
                keys are the names of variables to store, and the values are functions
                called to retrieve these variables. These functions must accept a single
                input, an integer with the time in nanoseconds when the function is
                called.
            min_log_period (int, optional): The minimum interval between data recordings
                Defaults to 0.
        """
        super().__init__()

        if min_log_period < 0:
            raise ValueError("min_log_period must be greater than or equal to 0")

        self.logging_functions = logging_functions

        self.min_log_period = min_log_period
        self._next_update_time = 0
        self.clear()

    def clear(self):
        """Called to clear the internal data storages"""
        self._times = []
        self._variables = {name: [] for name in self.logging_functions}

    def times(self):
        """Retrieve the times when the data was logged"""
        return np.array(self._times)

    def _get_logged_variable(self, name: str) -> np.ndarray:
        """Return the logged samples for ``name``.

        :param name: Logged variable name.
        :return: Logged samples for ``name``.
        :raises AttributeError: If the internal storage is unavailable.
        :raises KeyError: If ``name`` is not being logged.
        """
        try:
            variables = object.__getattribute__(self, "_variables")
        except AttributeError as err:
            raise AttributeError(
                f"'{type(self).__name__}' object has no attribute '{name}'"
            ) from err

        if name in variables:
            return np.array(variables[name])

        raise KeyError(
            f"Logger is not logging '{name}'. Must be one of: {', '.join(variables)}"
        )

    def __getitem__(self, name: str) -> np.ndarray:
        """Return logged samples for any stored variable name.

        This accessor supports names that cannot be exposed as attributes,
        including invalid identifiers and keys that collide with class members.

        :param name: Logged variable name.
        :return: Logged samples for ``name``.
        """
        return self._get_logged_variable(name)

    def Reset(self, CurrentSimNanos):
        self.clear()
        self._next_update_time = CurrentSimNanos
        return super().Reset(CurrentSimNanos)

    def UpdateState(self, CurrentSimNanos):
        if CurrentSimNanos >= self._next_update_time:
            self._times.append(CurrentSimNanos)
            for variable_name, logging_function in self.logging_functions.items():
                try:
                    val = logging_function(CurrentSimNanos)
                except Exception as ex:
                    self.bskLogger.bskLog(sysModel.BSK_ERROR,
                                        f"Error while logging '{variable_name}'"
                                        f" in logger '{self.ModelTag}': {ex}")
                    val = None
                val = np.array(val).squeeze()
                self._variables[variable_name].append(val)

            self._next_update_time += self.min_log_period
        return super().UpdateState(CurrentSimNanos)

    def __getattr__(self, name: str) -> Any:
        """Return logged data for attributes that are not explicit class members.

        This fallback supports logger names that cannot be exposed safely as
        Python properties, such as invalid identifiers. Use ``logger[name]``
        for logged names that collide with existing class attributes.
        """
        try:
            return self._get_logged_variable(name)
        except KeyError as err:
            raise AttributeError(err.args[0]) from err
