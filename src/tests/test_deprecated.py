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

r"""
This test demonstrates how to deprecate functions, classes, attributes, 
and properties defined in Python code. To learn how to deprecate C++ code
exposed to users through SWIG, see:

    src\architecture\utilitiesSelfCheck\_UnitTest\swigDeprecatedCheck.i
"""
import pytest

# The following block represents the target code before deprecations:
"""
def standaloneFun(arg):
    ...

class PythonTest:

    def __init__(self) -> None:
        self.myAttribute = 0
        self.myPropertyInner = 0

    def myFun1(self):
        ...

    @property
    def myProperty(self):
        return self.myPropertyInner * 2
    
    @myProperty.setter
    def myProperty(self, value: int):
        self.myPropertyInner = value / 2
"""

# The following is the code with all elements deprecated
from Basilisk.utilities import deprecated


@deprecated.deprecated("2099/05/05", "Use standaloneFun_new() instead!")
def standaloneFun(arg):
    ...


@deprecated.deprecated("2099/05/05", "This entire class is replaced by RustTest")
class PythonTest:
    myAttribute = deprecated.DeprecatedAttribute(
        "2099/05/05", "myAttribute is no longer used in the simulation"
    )

    def __init__(self) -> None:
        with deprecated.ignore("myAttribute"):  # Prevents warnings here
            self.myAttribute = 0
        self.myPropertyInner = 0

    @deprecated.deprecated("2000/05/05", "myFun1 is super old!")
    def myFun1(self):
        ...

    @property
    def myProperty(self):
        return self.myPropertyInner * 2

    @myProperty.setter
    def myProperty(self, value: int):
        self.myPropertyInner = value / 2

    myProperty = deprecated.DeprecatedProperty(
        "2099/05/05", "myProperty is no longer used in the simulation", myProperty
    )


# This test shows how calling or using the classes/functions/attributes
# will raise warnings
def test_correctWarnings():
    """Shows how calling or using the deprecated classes/functions/attributes
    will raise warnings"""

    with pytest.warns(deprecated.BSKDeprecationWarning):
        standaloneFun(0)

    with pytest.warns(deprecated.BSKDeprecationWarning):
        test = PythonTest()

    with pytest.warns(deprecated.BSKDeprecationWarning):
        test.myAttribute = 5

    with pytest.warns(deprecated.BSKDeprecationWarning):
        stored = test.myAttribute
        assert stored == 5  # Check it still works despite being deprecated

    # Note the BSKUrgentDeprecationWarning because the removal date is passed
    with pytest.warns(deprecated.BSKUrgentDeprecationWarning):
        test.myFun1()

    with pytest.warns(deprecated.BSKDeprecationWarning):
        test.myProperty = 10

    with pytest.warns(deprecated.BSKDeprecationWarning):
        stored = test.myProperty
        assert stored == 10


def test_ignoreWarnings():
    """Checks that the ignoring deprecation functionality is working correctly"""
    import warnings

    # Here, we intend to supress all BSKDeprecationWarning, so any that are raised
    # should be reported as errors
    warnings.filterwarnings("error", category=deprecated.BSKDeprecationWarning)

    with deprecated.ignore("standaloneFun"):
        standaloneFun(0)

    with deprecated.ignore("PythonTest"):
        test = PythonTest()

    # This will ignore only calls to myAttribute to PythonTest
    with deprecated.ignore("PythonTest.myAttribute"):
        test.myAttribute = 5

    # This will ignore every call to any deprecated myAttribute (not only in PythonTest)
    with deprecated.ignore("myAttribute"):
        _ = test.myAttribute

    with deprecated.ignore("PythonTest.myProperty"):
        test.myProperty = 0

    with deprecated.ignore("myProperty"):
        _ = test.myProperty

    # BSKUrgentDeprecationWarning cannot be ignored!
    with pytest.warns(deprecated.BSKUrgentDeprecationWarning):
        with deprecated.ignore("myFun1"):  # does nothing
            test.myFun1()


if __name__ == "__main__":
    standaloneFun(0)
    testClass = PythonTest()
    testClass.myFun1()
    testClass.myAttribute = 1
    testClass.myProperty = 2
