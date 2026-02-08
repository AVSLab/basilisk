/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */
%module swig_deprecated

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();


/** Used to deprecate a function in C++ that is exposed to Python through SWIG.

'function' is the SWIG identifier of the function. If it is a standalone function,
then this is simply its name. If it's a class function, then this should be
[CLASS_NAME]::[FUNCTION_NAME]

'removal_date' is the expected removal date in the format 'YYYY/MM/DD'. Think of
an amount of time that would let users update their code, and then add that duration
to today's date to find a reasonable removal date.

'message' is a text that is directly shown to the users. Here, you may explain
why the function is deprecated, alternative functions, links to documentation
or scenarios that show how to translate deprecated code...

See src\architecture\utilitiesSelfCheck\swigDeprecatedCheck.i
*/
%define %deprecated_function(function, removal_date, message)
%pythonprepend function %{
    from Basilisk.utilities import deprecated
    deprecated.deprecationWarn(f"{__name__}.function".replace("::","."), `removal_date`, `message`)
%}
%enddef

/** Used to deprecate a public class variable in C++ that is exposed to Python through SWIG.

'class' is the SWIG identifier of the class.

'variable' is the name of the variable.

'removal_date' is the expected removal date in the format 'YYYY/MM/DD'. Think of
an amount of time that would let users update their code, and then add that duration
to today's date to find a reasonable removal date.

'message' is a text that is directly shown to the users. Here, you may explain
why the variable is deprecated, alternative variables, links to documentation
or scenarios that show how to translate deprecated code...

See src\architecture\utilitiesSelfCheck\swigDeprecatedCheck.i
*/
%define %deprecated_variable(class, variable, removal_date, message)
%extend class {
    %pythoncode %{
    from Basilisk.utilities import deprecated
    variable = deprecated.DeprecatedProperty(`removal_date`, `message`, variable)
    %}
}
%enddef

/** Used to deprecate a structure in C++ that is exposed to Python through SWIG. This is specifically
for aliasing, where a structure is renamed and support for both names is desired. Fields can also be
aliased within structures.

Usage within .i interface files:

- Renaming 'MultiSphere' to 'MultiShape':

%pythoncode %{
import sys
mod = sys.modules[__name__]
mod.MultiSphere = _DeprecatedWrapper(
    mod.MultiShape,
    aliasName="MultiSphere",
    targetName="MultiShape",
    removalDate="2026/02/21"
)
protectAllClasses(sys.modules[__name__])
%}

- Renaming 'radius' to 'dimensions':

%pythoncode %{
import sys
mod = sys.modules[__name__]
mod.MultiShape = _DeprecatedWrapper(
    mod.MultiShape,
    targetName="MultiShape",
    deprecatedFields={"radius": "dimensions"},
    typeConversion="scalarTo3D"
    removalDate="2026/02/21"
)
protectAllClasses(sys.modules[__name__])
%}

*/
%pythoncode %{
import sys
from Basilisk.utilities import deprecated

class _DeprecatedWrapper:
    def __init__(self, target, aliasName=None,
                               targetName=None,
                               deprecatedFields=None,
                               typeConversion=None,
                               removalDate=None):
        self._target = target
        self._aliasName = aliasName
        self._targetName = targetName
        self._deprecatedFields = deprecatedFields or {}
        self._typeConversion = typeConversion
        self._removalDate = removalDate

    def __call__(self, *args, **kwargs):
        if self._aliasName:
            deprecated.deprecationWarn(self._aliasName, self._removalDate, f"Use '{self._targetName}' instead.")

        instance = self._target(*args, **kwargs)

        for old_attr, new_attr in self._deprecatedFields.items():
            if hasattr(instance, new_attr):  # Ensure new attribute exists
                _inject_deprecated_property(instance, old_attr, new_attr, self._removalDate, self._typeConversion)

        return instance  # Always return the original instance

def _inject_deprecated_property(instance, old_attr, new_attr, removal_date, typeConversion=None):
    def getter(self):
        deprecated.deprecationWarn(old_attr, removal_date, f"Use '{new_attr}' instead.")
        return getattr(self, new_attr)

    def setter(self, value):
        deprecated.deprecationWarn(old_attr, removal_date, f"Use '{new_attr}' instead.")
        # Conduct type conversions if new variable type is changed
        if typeConversion and typeConversion == "scalarTo3D":
            setattr(self, new_attr, [value, value, value])
        elif typeConversion and typeConversion == "useDefaultDouble":
            setattr(self, new_attr, -1.0)
        else:
            setattr(self, new_attr, value)

    setattr(instance.__class__, old_attr, property(getter, setter))

%}
