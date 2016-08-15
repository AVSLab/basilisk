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
    $result = PyList_New($1_dim0);
    //PyObject *outObject = 0;
    for (i = 0; i < $1_dim0; i++) {
        //SWIG_ConvertPtr(SWIG_as_voidptr(&($1[i])), locOutObj, SWIGTYPE_p_ ## type, 0 |  0 );
        PyObject *locOutObj = SWIG_NewPointerObj(SWIG_as_voidptr(&($1[i])), SWIGTYPE_p_ ## type, 0 |  0 );
        
        if(PyNumber_Check(locOutObj)){
            PyObject *outObject = PyFloat_FromDouble((double) $1[i]);
            PyList_SetItem($result,i,outObject);
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

%define SCOTTROX(type)
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

%array_functions(double, doubleArray);
%array_functions(long, longArray);
%array_functions(int, intArray);
%array_functions(short, shortArray);

%pythoncode %{

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
        except (AttributeError, TypeError) as e:
            pass
    
%}


