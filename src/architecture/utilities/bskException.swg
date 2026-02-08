%include "exception.i"

%{
#include "architecture/utilities/bskLogging.h"

PyObject* get_PyExc_BasiliskError() {
    PyObject* bskLogging = PyImport_ImportModule("Basilisk.architecture.bskLogging");

    PyObject* PyExc_BasiliskError = PyObject_GetAttrString(bskLogging, "BasiliskError");
    Py_DECREF(bskLogging);

    return PyExc_BasiliskError;
}
%}

%define %bsk_exception_body(CATCH...)
    try {
        $action
    } catch (const BasiliskError& e) {
        PyErr_SetString(get_PyExc_BasiliskError(), const_cast<char*>(e.what()));
        SWIG_fail;
    }
    CATCH
%enddef


// This macro can be used to add other exception handling logic to the swig interface
// without overriding the bsk_exception logic, since %exception directives cannot nest.
// It should be placed after the %default_bsk_exception and define any additional CATCH
// statements.
%define %bsk_exception(FUNC, CATCH)
%exception FUNC {
    %bsk_exception_body(CATCH)
}
%enddef


// This macro should be in any .i file that should return exceptions to Python. CATCH
// can be used to define additional exception handling for all functions in the file.
%define %default_bsk_exception(CATCH...)
%exception {
    %bsk_exception_body(CATCH)
}
%enddef
