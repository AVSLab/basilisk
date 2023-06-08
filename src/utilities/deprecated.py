#
#  ISC License
#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import warnings
from warnings import catch_warnings as catch_warnings
import datetime
import types
from typing import Union, Literal, TYPE_CHECKING, Any


class BSKDeprecationWarning(Warning):
    __color__ = "\x1b[33;20m"  # Yellow


class BSKUrgentDeprecationWarning(Warning):
    __color__ = "\x1b[31;1m"  # Bold red


def filterwarnings(
    action: Literal["error", "ignore", "always", "default", "module", "once"],
    identifier: str,
    **kwargs: Any,
):
    """Use this function to create global deprecation warning filters.

    The most common use of this function is suppressing warnings for deprecated systems
    that you still intend to use. Imagine you want to ignore deprecation warnings
    for the method `myMethod` in class `ValidClass` in module `my_module`. You would add
    the following line to your simulation script::

        deprecated.filterwarnings("ignore", "my_module.ValidClass.myMethod")

    Note that deprecation warnings should not be ignored blindly. Deprecated code WILL
    be removed in future releases, so use warning suppression carefully.

    Args:
        action (Literal["error", "ignore", "always", "default", "module", "once"]):
            Controls how the filtered warnings will behave.
        identifier (str): The warning message must contain this string. This can be
            the function or variable identifier (qualified name) in order to hide
            deprecation warnings for specific features.
        **kwargs (Any): Any other keyword arguments are passed directly to
            `warnings.filterwarnings`.
    """
    warnings.filterwarnings(
        action, f".*{identifier}.*", category=BSKDeprecationWarning, **kwargs
    )


class ignore:
    """Use this context manager to ignore warnings in a precise section of code.

    The most common use of this function is suppressing warnings for specific calls.
    Imagine you want to ignore deprecation warnings for the method `myMethod` in
    class `ValidClass` in module `my_module` for a single call to this function.
    You would do::

        myClass = my_module.ValidClass()

        with deprecated.ignore("my_module.ValidClass.myMethod"):
            myClass.myMethod() # Does not raise a warning

        myClass.myMethod() # Raises a warning

    Note that deprecation warnings should not be ignored blindly. Deprecated code WILL
    be removed in future releases, so use warning suppression carefully.
    """

    def __init__(self, identifier: str) -> None:
        self.catch = catch_warnings()
        self.identifier = identifier

    def __enter__(self):
        self.catch.__enter__()
        filterwarnings("ignore", self.identifier)

    def __exit__(self, *exc_info):
        self.catch.__exit__(*exc_info)


def deprecationWarn(id: str, removalDate: Union[datetime.date, str], msg: str):
    """Utility function to raise deprecation warnings inside a function body.

    This function should rarely be used on its own. For deprecated Python code,
    prefer the `@deprecated` decorator. For deprecated C++ code, use the SWIG
    macros in `swig_deprecated.i`.

    Args:
        id (str): An identifier for the deprecated feature (function/variable
            qualified name)
        removalDate (Union[datetime.date, str]): The date when we expect to remove this
            deprecated feature, in the format 'YYYY/MM/DD' or as a ``datetime.date``.
            Think of an amount of time that would let users update their code, and then
            add that duration to today's date to find a reasonable removal date.
        msg (str, optional): a text that is directly shown to the users. Here, you may
            explain why the function is deprecated, alternative functions, links to
            documentation or scenarios that show how to translate deprecated code...
    """

    id = id.replace(")", "").replace("(", "")
    if isinstance(removalDate, str):
        removalDate = datetime.datetime.strptime(removalDate, r"%Y/%m/%d").date()

    if removalDate > datetime.date.today():
        warnings.warn(
            f"{id} will be removed after {removalDate}: {msg}",
            category=BSKDeprecationWarning,
            stacklevel=3,
        )
    else:
        warnings.warn(
            f"{id} will be removed soon: {msg}",
            category=BSKUrgentDeprecationWarning,
            stacklevel=3,
        )


def deprecated(removalDate: str, msg: str):
    """A decorator for functions or classes that are deprecated.

    Usage::

        @deprecated.deprecated("2024/05/28", "Use MyNewClass")
        class MyClass:
            ...

    or::

        @deprecated.deprecated("2024/05/28", "myFun is unsafe now")
        def myFun(a, b):
            ...

    or::

        class ValidClass:

            @deprecated.deprecated("2024/05/28", "myMethod is slow, use myBetterMethod")
            def myMethod(self, a, b):
                ...

    Args:
        removalDate (Union[datetime.date, str]): The date when we expect to remove this
            deprecated feature, in the format 'YYYY/MM/DD' or as a ``datetime.date``.
            Think of an amount of time that would let users update their code, and then
            add that duration to today's date to find a reasonable removal date.
        msg (str, optional): a text that is directly shown to the users. Here, you may
            explain why the function is deprecated, alternative functions, links to
            documentation or scenarios that show how to translate deprecated code...
    """

    def wrapper(func):
        id = f"{func.__module__}.{func.__qualname__}"

        def inner_wrapper(*args, **kwargs):
            deprecationWarn(id, removalDate, msg)
            return func(*args, **kwargs)

        return inner_wrapper

    return wrapper


class DeprecatedAttribute:
    """Use this descriptor to deprecate class attributes (variables).

    If you want to deprecate ``myAttribute`` in the following class::

        class MyClass:

            def __init__(self) -> None:
                self.myAttribute = 0

    You can use the following syntax::

        class PythonTest:

            myAttribute = deprecated.DeprecatedAttribute("2099/05/05", "myAttribute is no longer used in the simulation")

            def __init__(self) -> None:
                with deprecated.ignore("myAttribute"): # Prevents warnings here
                    self.myAttribute = 0

    The attribute will work as before, but deprecation warnings will be raised
    everytime it's used (read or set).
    """

    def __init__(self, removalDate: str, msg: str) -> None:
        self.removalDate = removalDate
        self.msg = msg

    def __set_name__(self, owner, name):
        self.id = f"{owner.__module__}.{owner.__qualname__}.{name}"
        self.name = name

    def __get__(self, obj, objtype=None):
        deprecationWarn(self.id, self.removalDate, self.msg)
        return getattr(obj, f"_{self.name}")

    def __set__(self, obj, value):
        deprecationWarn(self.id, self.removalDate, self.msg)
        setattr(obj, f"_{self.name}", value)


class DeprecatedProperty:  # type: ignore
    """Use this descriptor to deprecate class properties.

    If you want to deprecate ``myProperty`` in the following class::

        class MyClass:

            @property
            def myProperty(self):
                return 0

            @myProperty.setter
            def myProperty(self, value: int):
                ...

    You can use the following syntax::

        class PythonTest:

            @property
            def myProperty(self):
                return 0

            @myProperty.setter
            def myProperty(self, value: int):
                ...

            myProperty = deprecated.DeprecatedProperty(
                "2099/05/05",
                "myProperty is no longer used in the simulation",
                myProperty)

    The property will work as before, but deprecation warnings will be raised
    everytime it's used (read or set).
    """

    def __init__(self, removalDate: str, msg: str, property: property) -> None:
        self.removalDate = removalDate
        self.msg = msg
        self.property = property

        if not hasattr(self.property, "__get__") or not hasattr(
            self.property, "__set__"
        ):
            raise ValueError("DeprecatedProperty must be given an existing property")

    def __set_name__(self, owner, name):
        self.id = f"{owner.__module__}.{owner.__qualname__}.{name}"

    def __get__(self, *args, **kwargs):
        deprecationWarn(self.id, self.removalDate, self.msg)
        return self.property.__get__(*args, **kwargs)

    def __set__(self, *args, **kwargs):
        deprecationWarn(self.id, self.removalDate, self.msg)
        self.property.__set__(*args, **kwargs)

    def __delete__(self, *args, **kwargs):
        self.property.__delete__(*args, **kwargs)


# Typing hack so that linters don't complain about redefining properties
if TYPE_CHECKING:

    def DeprecatedProperty(removalDate: str, msg: str) -> Any:  # type: ignore
        ...


# Monkey patching showwarning to modify the behaviour for our warnings
def formatwarning(
    message,
    category,
    filename,
    lineno,
    line=None,
    __defaultformatwarning=warnings.formatwarning,
):
    """Hook to write a warning to a file; replace if you like."""
    if issubclass(category, (BSKDeprecationWarning, BSKUrgentDeprecationWarning)):
        message = category(str(message) + "\x1b[0m")  # Append the color reset character

        # Produce a "fake object" with sole attribute __name__ which is the name of
        # the actual category prepended by the color character
        category = types.SimpleNamespace(
            __name__=category.__color__ + category.__name__
        )

    return __defaultformatwarning(message, category, filename, lineno, line)


warnings.formatwarning = formatwarning
