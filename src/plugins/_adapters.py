from __future__ import annotations

from typing import Any, Callable

from Basilisk.architecture import sysModel


def normalize_factory(obj: Any) -> Callable[[], Any]:
    create_factory = getattr(obj, "create_factory", None)
    if callable(create_factory):
        f = create_factory()
        if not callable(f):
            raise TypeError("create_factory() must return a callable factory")
        return f

    if callable(obj):
        return obj

    raise TypeError("Factory must be a callable or module/object with create_factory()")


class PybindPluginAsSysModel(sysModel.SysModel):
    """
    Adapter that makes a pybind plugin instance look like a Basilisk SysModel.
    """

    def __init__(self, impl: Any):
        super().__init__()
        self._impl = impl

        tag = getattr(impl, "ModelTag", None) or impl.__class__.__name__
        self.ModelTag = str(tag)

        if not callable(getattr(impl, "Reset", None)):
            raise TypeError(
                f"Plugin instance {impl!r} does not define callable Reset(t)"
            )
        if not callable(getattr(impl, "UpdateState", None)):
            raise TypeError(
                f"Plugin instance {impl!r} does not define callable UpdateState(...)"
            )

    def __getattr__(self, name: str):
        return getattr(self._impl, name)

    def Reset(self, current_sim_nanos: int) -> None:
        self._impl.Reset(current_sim_nanos)

    def UpdateState(self, *args) -> None:
        if len(args) == 1:
            self._impl.UpdateState(args[0])
            return
        if len(args) == 2:
            try:
                self._impl.UpdateState(args[0], args[1])
            except TypeError:
                self._impl.UpdateState(args[0])
            return
        raise TypeError(f"UpdateState expected 1 or 2 args, got {len(args)}")
