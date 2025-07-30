%include "exception.i"

%define %bsk_exception_body(CATCH...)
    try {
        $action
    } catch (const std::runtime_error& e) {
        SWIG_exception(SWIG_RuntimeError, e.what());
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
