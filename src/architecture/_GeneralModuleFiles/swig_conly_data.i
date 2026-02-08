/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

%module swig_conly_data

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();


%include "stdint.i"

%define ARRAYASLIST(type, fromfunc, asfunc)
%typemap(in) type [ANY] (type temp[$1_dim0]) {
    int i;
    void *blankPtr = 0 ;
    int resOut = 0 ;
    if (!PySequence_Check($input)) {
        PyErr_SetString(PyExc_ValueError,"Expected a sequence");
        return NULL;
    }
    if (PySequence_Length($input) > $1_dim0) {
        printf("Value: %d\n", $1_dim0);
        PyErr_SetString(PyExc_ValueError,"Size mismatch. Expected $1_dim0 elements");
        return NULL;
    }
    memset(temp, 0x0, $1_dim0*sizeof(type));
    for (i = 0; i < PySequence_Length($input); i++) {
        PyObject *o = PySequence_GetItem($input,i);
        if (PyNumber_Check(o)) {
            temp[i] = (type)asfunc(o);
        } else {
            resOut = SWIG_ConvertPtr(o, &blankPtr,$1_descriptor, 0 |  0 );
            if (!SWIG_IsOK(resOut)) {
                SWIG_exception_fail(SWIG_ArgError(resOut), "Could not convert that type into a pointer for some reason.  This is an ugly SWIG failure.  Good luck.\n");
                Py_DECREF(o);
                return NULL;
            }
            memcpy(&(temp[i]), blankPtr, sizeof(type));
        }
        Py_DECREF(o);
    }
    $1 = temp;
}

%typemap(memberin) type [ANY] {
    int i;
    for (i = 0; i < $1_dim0; i++) {
        memcpy(&($1[i]), &($input[i]), sizeof(type));
    }
}

%typemap(out) type [ANY] {
    int i;
    $result = PyList_New(0);
    PyObject *locOutObj = 0;
    for (i = 0; i < $1_dim0; i++) {
        locOutObj = SWIG_NewPointerObj(SWIG_as_voidptr(&($1[i])), $1_descriptor, 0 |  0 );

        if(PyNumber_Check(locOutObj)){
            PyObject *outObject = fromfunc($1[i]);
            PyList_Append($result,outObject);
            Py_DECREF(outObject);
            Py_DECREF(locOutObj);
        }
        else
        {
            PyList_SetItem($result, i, locOutObj);
            // NOTE: SetItem steals the reference, so Py_DECREF is unnecessary
        }
    }
}

%enddef
ARRAYASLIST(int, PyLong_FromLong, PyLong_AsLong)
ARRAYASLIST(unsigned int, PyLong_FromUnsignedLong, PyLong_AsUnsignedLong)
ARRAYASLIST(int32_t, PyLong_FromLong, PyLong_AsLong)
ARRAYASLIST(uint32_t, PyLong_FromUnsignedLong, PyLong_AsUnsignedLong)
ARRAYASLIST(long long, PyLong_FromLongLong, PyLong_AsLongLong)
ARRAYASLIST(unsigned long long, PyLong_FromUnsignedLongLong, PyLong_AsUnsignedLongLong)
ARRAYASLIST(int64_t, PyLong_FromLongLong, PyLong_AsLongLong)
ARRAYASLIST(uint64_t, PyLong_FromUnsignedLongLong, PyLong_AsUnsignedLongLong)
ARRAYASLIST(short, PyLong_FromLong, PyLong_AsLong)
ARRAYASLIST(unsigned short, PyLong_FromUnsignedLong, PyLong_AsUnsignedLong)
ARRAYASLIST(int16_t, PyLong_FromLong, PyLong_AsLong)
ARRAYASLIST(uint16_t, PyLong_FromUnsignedLong, PyLong_AsUnsignedLong)
ARRAYASLIST(size_t, PyLong_FromSize_t, PyLong_AsSize_t)
ARRAYASLIST(ssize_t, PyLong_FromSsize_t, PyLong_AsSsize_t)
ARRAYASLIST(double, PyFloat_FromDouble, PyFloat_AsDouble)
ARRAYASLIST(float, PyFloat_FromDouble, PyFloat_AsDouble)
ARRAYASLIST(uint8_t, PyLong_FromUnsignedLong, PyLong_AsUnsignedLong)
ARRAYASLIST(unsigned char, PyLong_FromUnsignedLong, PyLong_AsUnsignedLong)
ARRAYASLIST(int8_t, PyLong_FromLong, PyLong_AsLong)
ARRAYASLIST(signed char, PyLong_FromLong, PyLong_AsLong)
ARRAYASLIST(bool, PyBool_FromLong, PyObject_IsTrue)

%define ARRAY2ASLIST(type, fromfunc, asfunc)

%typemap(in) type [ANY][ANY] (type temp[$1_dim0][$1_dim1]) {
    Py_ssize_t i;
    if(!PySequence_Check($input)) {
        PyErr_SetString(PyExc_ValueError,"Expected a list of lists!  Does not appear to be that.");
        return NULL;
    }
    Py_ssize_t rowLength = 0;
    for(i=0; i<PySequence_Length($input); i++){
        PyObject *obj = PySequence_GetItem($input, i);
        if(!PySequence_Check(obj)) {
            printf("Row bad in matrix: %zd\n", i);
            PyErr_SetString(PyExc_ValueError,"Need a list for each row");
        }
        if(!rowLength)
        {
            rowLength = PySequence_Length(obj);
        }
        if(!rowLength || rowLength != PySequence_Length(obj))
        {
            printf("Row bad in matrix: %zd\n", i);
            PyErr_SetString(PyExc_ValueError,"All rows must be the same length!");
        }
        int j;
        for(j=0; j<rowLength; j++)
        {
            PyObject *o = PySequence_GetItem(obj, j);
            temp[i][j] = (type)asfunc(o);
            Py_DECREF(o);
        }
        Py_DECREF(obj);
    }
    $1 = temp;
}

%typemap(memberin) type [ANY][ANY] {
    int i, j;
    for (i = 0; i < $1_dim0; i++) {
        for(j=0; j < $1_dim1; j++) {
            memcpy(&($1[i][j]), &($input[i][j]), sizeof(type));
        }
    }
}

%typemap(out) type [ANY][ANY] {
    int i, j;
    $result = PyList_New(0);
    for(i=0; i<$1_dim0; i++)
    {
        PyObject *locRow = PyList_New(0);
        for(j=0; j<$1_dim1; j++)
        {
            PyObject *outObject = fromfunc($1[i][j]);
            PyList_Append(locRow, outObject);
            Py_DECREF(outObject);
        }
        PyList_Append($result, locRow);
        Py_DECREF(locRow);
    }
}

%enddef
ARRAY2ASLIST(int, PyLong_FromLong, PyLong_AsLong)
ARRAY2ASLIST(unsigned int, PyLong_FromUnsignedLong, PyLong_AsUnsignedLong)
ARRAY2ASLIST(int32_t, PyLong_FromLong, PyLong_AsLong)
ARRAY2ASLIST(uint32_t, PyLong_FromUnsignedLong, PyLong_AsUnsignedLong)
ARRAY2ASLIST(long long, PyLong_FromLongLong, PyLong_AsLongLong)
ARRAY2ASLIST(unsigned long long, PyLong_FromUnsignedLongLong, PyLong_AsUnsignedLongLong)
ARRAY2ASLIST(int64_t, PyLong_FromLongLong, PyLong_AsLongLong)
ARRAY2ASLIST(uint64_t, PyLong_FromUnsignedLongLong, PyLong_AsUnsignedLongLong)
ARRAY2ASLIST(short, PyLong_FromLong, PyLong_AsLong)
ARRAY2ASLIST(unsigned short, PyLong_FromUnsignedLong, PyLong_AsUnsignedLong)
ARRAY2ASLIST(int16_t, PyLong_FromLong, PyLong_AsLong)
ARRAY2ASLIST(uint16_t, PyLong_FromUnsignedLong, PyLong_AsUnsignedLong)
ARRAY2ASLIST(size_t, PyLong_FromSize_t, PyLong_AsSize_t)
ARRAY2ASLIST(ssize_t, PyLong_FromSsize_t, PyLong_AsSsize_t)
ARRAY2ASLIST(double, PyFloat_FromDouble, PyFloat_AsDouble)
ARRAY2ASLIST(float, PyFloat_FromDouble, PyFloat_AsDouble)
ARRAY2ASLIST(uint8_t, PyLong_FromUnsignedLong, PyLong_AsUnsignedLong)
ARRAY2ASLIST(unsigned char, PyLong_FromUnsignedLong, PyLong_AsUnsignedLong)
ARRAY2ASLIST(int8_t, PyLong_FromLong, PyLong_AsLong)
ARRAY2ASLIST(signed char, PyLong_FromLong, PyLong_AsLong)
ARRAY2ASLIST(bool, PyBool_FromLong, PyObject_IsTrue)

%define STRUCTASLIST(type)
%typemap(in) type [ANY] (type temp[$1_dim0]) {
    int i;
    void *blankPtr = 0 ;
    int resOut = 0 ;
    if (!PySequence_Check($input)) {
        PyErr_SetString(PyExc_ValueError,"Expected a sequence");
        return NULL;
    }
    if (PySequence_Length($input) > $1_dim0) {
        printf("Value: %d\n", $1_dim0);
        PyErr_SetString(PyExc_ValueError,"Size mismatch. Expected $1_dim0 elements");
        return NULL;
    }
    memset(temp, 0x0, $1_dim0*sizeof(type));
    for (i = 0; i < PySequence_Length($input); i++) {
        PyObject *o = PySequence_GetItem($input,i);
        resOut = SWIG_ConvertPtr(o, &blankPtr,$1_descriptor, 0 |  0 );
        if (!SWIG_IsOK(resOut)) {
            SWIG_exception_fail(SWIG_ArgError(resOut), "Could not convert that type into a pointer for some reason.  This is an ugly SWIG failur Good luck.\n");
            Py_DECREF(o);
            return NULL;
        }
        memcpy(&(temp[i]), blankPtr, sizeof(type));
        Py_DECREF(o);
    }
    $1 = temp;
}

%typemap(memberin) type [ANY] {
    int i;
    for (i = 0; i < $1_dim0; i++) {
        memcpy(&($1[i]), &($input[i]), sizeof(type));
    }
}

%typemap(out) type [ANY] {
    int i;
    $result = PyList_New($1_dim0);
    for (i = 0; i < $1_dim0; i++) {
        PyObject *locOutObj = SWIG_NewPointerObj(SWIG_as_voidptr(&($1[i])), $1_descriptor, 0 |  0 );
        PyList_SetItem($result, i, locOutObj);
        // NOTE: SetItem steals the reference, so Py_DECREF is unnecessary
    }
}
%enddef

%include "carrays.i"
%include "cmalloc.i"
#define GEN_SIZEOF(type) %sizeof(type, type)

%array_functions(double, doubleArray);
%array_functions(long, longArray);
%array_functions(int, intArray);
%array_functions(short, shortArray);

%pythoncode %{

def getStructSize(self):
    try:
        class_name = repr(self).split(';')[0].split('.')[-1]
        sizeof_variable_name = 'sizeof_' + class_name
        size = globals().get(sizeof_variable_name)

        if size is None:
            raise ValueError(f"{sizeof_variable_name} not found in globals()")
    except (NameError) as e:
        typeString = 'sizeof_' + repr(self).split(';')[0].split('.')[-1]
        raise NameError(e.message + '\nYou tried to get this size macro: ' + typeString +
            '\n It appears to be undefined.  \nYou need to run the SWIG GEN_SIZEOF' +
            ' SWIG macro against the class/struct in your SWIG file if you want to ' +
            ' make this call.\n')


def protectSetAttr(self, name, value):
    if(hasattr(self, name) or name == 'this' or name.find('swig') >= 0):
        object.__setattr__(self, name, value)
    else:
        raise ValueError('You tried to add this variable: ' + name + '\n' +
            'To this class: ' + str(self))

def protectAllClasses(moduleType):
    import inspect

    clsmembers = inspect.getmembers(moduleType, inspect.isclass)
    for member in clsmembers:
        try:
            member[1].__setattr__ = protectSetAttr
            member[1].getStructSize = getStructSize
        except (AttributeError, TypeError) as e:
            pass

%}
