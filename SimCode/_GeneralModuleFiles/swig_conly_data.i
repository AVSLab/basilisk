%include "stdint.i"
%typemap(in) double [ANY] (double temp[$1_dim0]) {
    int i;
    if (!PySequence_Check($input)) {
        PyErr_SetString(PyExc_ValueError,"Expected a sequence");
        return NULL;
    }
    if (PySequence_Length($input) > $1_dim0) {
        printf("Value: %d\n", $1_dim0);
        PyErr_SetString(PyExc_ValueError,"Size mismatch. Expected $1_dim0 elements");
        return NULL;
    }
    memset(temp, 0x0, $1_dim0*sizeof(double));
    for (i = 0; i < PySequence_Length($input); i++) {
        PyObject *o = PySequence_GetItem($input,i);
        if (PyNumber_Check(o)) {
            temp[i] = (double) PyFloat_AsDouble(o);
        } else {
            PyErr_SetString(PyExc_ValueError,"Sequence elements must be numbers");
            return NULL;
        }
    }
    $1 = temp;
}

%typemap(memberin) double [ANY] {
    int i;
    for (i = 0; i < $1_dim0; i++) {
        $1[i] = $input[i];
    }
}

%typemap(out) double [ANY] {
    int i;
    $result = PyList_New($1_dim0);
    for (i = 0; i < $1_dim0; i++) {
        PyObject *o = PyFloat_FromDouble((double) $1[i]);
        PyList_SetItem($result,i,o);
    }
}

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


