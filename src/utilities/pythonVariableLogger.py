import keyword
from typing import Callable, Any, Dict

import numpy as np

from Basilisk.architecture import sysModel

LoggingFunction = Callable[[int], Any]

class PythonVariableLogger(sysModel.SysModel):
    """Logs results of arbitrary functions keyed by name.

    Usage::

        log = PythonVariableLogger({
            "a": lambda CurrentSimNanos: CurrentSimNanos**2,
            "b": lambda CurrentSimNanos: CurrentSimNanos**3,
        })
        # run sim
        times        = log.times()  # array of timestamps
        timesSquared = log["a"]     # or log.a
        timesCubed   = log["b"]     # or log.b
    """

    def __init__(
        self,
        logging_functions: Dict[str, LoggingFunction],
        min_log_period: int = 0,
    ) -> None:
        if min_log_period < 0:
            raise ValueError("min_log_period must be >= 0")
        super().__init__()
        self.logging_functions = logging_functions
        self.min_log_period = min_log_period
        self._next_update_time = 0
        self.clear()

    def clear(self) -> None:
        self._times: list = []
        self._variables: Dict[str, list] = {k: [] for k in self.logging_functions}

    def times(self) -> np.ndarray:
        return np.array(self._times)

    def __getitem__(self, name: str) -> np.ndarray:
        try:
            variables = object.__getattribute__(self, "_variables")
        except AttributeError as err:
            raise AttributeError(
                f"'{type(self).__name__}' object has no attribute '{name}'"
            ) from err

        if name not in variables:
            raise KeyError(
                f"Logger is not logging '{name}'. "
                f"Available: {', '.join(variables)}"
            )
        return np.array(variables[name])

    def __getattr__(self, name: str) -> np.ndarray:
        # Only called for names not found through normal lookup
        try:
            return self[name]
        except KeyError as err:
            raise AttributeError(str(err)) from err

    def Reset(self, CurrentSimNanos: int) -> None:
        self.clear()
        self._next_update_time = CurrentSimNanos
        return super().Reset(CurrentSimNanos)

    def UpdateState(self, CurrentSimNanos: int) -> None:
        if CurrentSimNanos < self._next_update_time:
            return super().UpdateState(CurrentSimNanos)

        self._times.append(CurrentSimNanos)
        for name, fn in self.logging_functions.items():
            try:
                val = np.array(fn(CurrentSimNanos)).squeeze()
            except Exception as ex:
                self.bskLogger.bskLog(
                    sysModel.BSK_ERROR,
                    f"Error logging '{name}' in '{self.ModelTag}': {ex}",
                )
                val = None
            self._variables[name].append(val)

        self._next_update_time += self.min_log_period
        return super().UpdateState(CurrentSimNanos)
