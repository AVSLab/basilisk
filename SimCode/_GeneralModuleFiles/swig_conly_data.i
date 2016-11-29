%include "stdint.i"
%define ARRAYASLIST(type)
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
            temp[i] = (type) PyFloat_AsDouble(o);
        } else {
            resOut = SWIG_ConvertPtr(o, &blankPtr,SWIGTYPE_p_ ## type, 0 |  0 );
            if (!SWIG_IsOK(resOut)) {
                SWIG_exception_fail(SWIG_ArgError(resOut), "Could not convert that type into a pointer for some reason.  This is an ugly SWIG failure.  Good luck.\n");
                return NULL;
            }
            memcpy(&(temp[i]), blankPtr, sizeof(type));
        }
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
        locOutObj = SWIG_NewPointerObj(SWIG_as_voidptr(&($1[i])), SWIGTYPE_p_ ## type, 0 |  0 );
        
        if(PyNumber_Check(locOutObj)){
            PyObject *outObject = PyFloat_FromDouble((double) $1[i]);
            PyList_Append($result,outObject);
            Py_DECREF(outObject);
        }
        else
        {
            PyList_SetItem($result, i, locOutObj);
        }
    }
}

%enddef
ARRAYASLIST(double)
ARRAYASLIST(int)
ARRAYASLIST(float)
ARRAYASLIST(unsigned int)

%define ARRAY2ASLIST(type)

%typemap(in) type [ANY][ANY] (type temp[$1_dim0][$1_dim1]) {

    if(!PySequence_Check($input)) {
        PyErr_SetString(PyExc_ValueError,"Expected a list of lists!  Does not appear to be that.");
        return NULL;
    }
    int rowLength = 0;
    for(int i=0; i<PySequence_Length($input); i++){
        PyObject *obj = PySequence_GetItem($input, i);
        if(!PySequence_Check($input)) {
            printf("Row bad in matrix: %d\n", i);
            PyErr_SetString(PyExc_ValueError,"Need a list for each row");
        }
        if(!rowLength)
        {
            rowLength = PySequence_Length(obj);
        }
        if(!rowLength || rowLength != PySequence_Length(obj))
        {
            printf("Row bad in matrix: %d\n", i);
            PyErr_SetString(PyExc_ValueError,"All rows must be the same length!");
        }
        for(int j=0; j<rowLength; j++)
        {
            temp[i][j] = PyFloat_AsDouble(PySequence_GetItem(obj, j));
        }
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
            PyObject *outObject = PyFloat_FromDouble($1[i][j]);
            PyList_Append(locRow, outObject);
        }
        PyList_Append($result, locRow);
    }
}

%enddef

ARRAY2ASLIST(double)
ARRAY2ASLIST(int)
ARRAY2ASLIST(float)
ARRAY2ASLIST(unsigned int)

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
        resOut = SWIG_ConvertPtr(o, &blankPtr,SWIGTYPE_p_ ## type, 0 |  0 );
        if (!SWIG_IsOK(resOut)) {
            SWIG_exception_fail(SWIG_ArgError(resOut), "Could not convert that type into a pointer for some reason.  This is an ugly SWIG failur Good luck.\n");
            return NULL;
        }
        memcpy(&(temp[i]), blankPtr, sizeof(type));
        
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
        PyObject *locOutObj = SWIG_NewPointerObj(SWIG_as_voidptr(&($1[i])), SWIGTYPE_p_ ## type, 0 |  0 );
        PyList_SetItem($result, i, locOutObj);
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
    return eval('sizeof_' + repr(self).split(';')[0].split('.')[-1])

def protectSetAttr(self, name, value):
    if(hasattr(self, name) or name == 'this'):
        object.__setattr__(self, name, value)
    else:
        raise ValueError('You tried to add this variable: ' + name + '\n' + \
            'To this class: ' + str(self))

def protectAllClasses(moduleType):
    import inspect
    clsmembers = inspect.getmembers(sys.modules[__name__], inspect.isclass)
    for member in clsmembers:
        try:
            exec(str(member[0]) + '.__setattr__ = protectSetAttr')
            exec(str(member[0]) + '.getStructSize = getStructSize') 
        except (AttributeError, TypeError) as e:
            pass
    
%}


