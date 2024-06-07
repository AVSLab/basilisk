
from Basilisk.moduleTemplates import cModuleTemplate
from Basilisk.moduleTemplates import cppModuleTemplate

import numpy.testing as npt

def test_protectAllClassesRaises():
    mod1 = cModuleTemplate.cModuleTemplate()
    mod2 = cppModuleTemplate.CppModuleTemplate()

    expected_ex = "You tried to add this variable(.*)"

    with npt.assert_raises_regex(ValueError, expected_ex):
        mod1.error = "error"

    with npt.assert_raises_regex(ValueError, expected_ex):
        mod2.error = "error"

if __name__ == "__main__":
    test_protectAllClassesRaises()
