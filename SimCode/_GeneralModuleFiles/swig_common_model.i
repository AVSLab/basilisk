%include "std_vector.i"
%include "std_string.i"
%include "std_set.i"
%include "std_pair.i"
%include "swig_conly_data.i"

%array_functions(bool, boolArray);

// Instantiate templates used by example
namespace std {
   %template(IntVector) vector<int>;
   %template(DoubleVector) vector<double>;
   %template(StringVector) vector<string>;
   %template(StringSet) set<string>;
   %template(intSet) set<unsigned long>;
   %template(ConstCharVector) vector<const char*>;
   %template(MultiArray) vector < vector <double>>;
}

%define EIGEN_MAT_WRAP(type)

%typemap(in) type {
    #include <Eigen/Dense>

    if(!PySequence_Check($input)) {
        PyErr_SetString(PyExc_ValueError,"Expected a list of lists!  Does not appear to be that.");
        return NULL;
    }
    int rowLength = 0;
    Eigen::MatrixXd matrixAssemble;
    for(int i=0; i<PySequence_Length($input); i++){
        PyObject *obj = PySequence_GetItem($input, i);
        if(!PySequence_Check($input)) {
            printf("Row bad in matrix: %d\n", i);
            PyErr_SetString(PyExc_ValueError,"Need a list for each row");
        }
        if(!rowLength)
        {
            rowLength = PySequence_Length(obj);
            matrixAssemble.resize(PySequence_Length($input), rowLength);
            matrixAssemble.fill(0.0);
        }
        if(!rowLength || rowLength != PySequence_Length(obj))
        {
            printf("Row bad in matrix: %d\n", i);
            PyErr_SetString(PyExc_ValueError,"All rows must be the same length!");
        }
        for(int j=0; j<rowLength; j++)
        {
            matrixAssemble(i, j) = PyFloat_AsDouble(PySequence_GetItem(obj, j));
        }
    }
    $1 = (matrixAssemble);
}

%typemap(in) type & {
    #include <Eigen/Dense>

    if(!PySequence_Check($input)) {
        PyErr_SetString(PyExc_ValueError,"Expected a list of lists!  Does not appear to be that.");
        return NULL;
    }
    int rowLength = 0;
    Eigen::MatrixXd matrixAssemble;
    for(int i=0; i<PySequence_Length($input); i++){
        PyObject *obj = PySequence_GetItem($input, i);
        if(!PySequence_Check($input)) {
            printf("Row bad in matrix: %d\n", i);
            PyErr_SetString(PyExc_ValueError,"Need a list for each row");
        }
        if(!rowLength)
        {
            rowLength = PySequence_Length(obj);
            matrixAssemble.resize(PySequence_Length($input), rowLength);
            matrixAssemble.fill(0.0);
        }
        if(!rowLength || rowLength != PySequence_Length(obj))
        {
            printf("Row bad in matrix: %d\n", i);
            PyErr_SetString(PyExc_ValueError,"All rows must be the same length!");
        }
        for(int j=0; j<rowLength; j++)
        {
            matrixAssemble(i, j) = PyFloat_AsDouble(PySequence_GetItem(obj, j));
        }
    }
	type localConvert = matrixAssemble;
    $1 = &(localConvert);
}

%typemap(typecheck) type {
    $1 = PySequence_Check($input);
}

%typemap(memberin) type {
    $1 = $input;
}

%typemap(out) type {
    int i, j;
    #include <Eigen/Dense>
    $result = PyList_New(0);
    Eigen::MatrixXd readPtr = ($1);
    for(i=0; i<readPtr.innerSize(); i++)
    {
        PyObject *locRow = PyList_New(0);
        for(j=0; j<readPtr.outerSize(); j++)
        {
            PyObject *outObject = PyFloat_FromDouble((readPtr)(i,j));
            PyList_Append(locRow, outObject);
        }
        PyList_Append($result, locRow);
    }
}

%typemap(out) type * {
    int i, j;
    #include <Eigen/Dense>
    $result = PyList_New(0);
    
    if(!($1))
    {
        return Py_None;
    }
	Eigen::MatrixXd readPtr = *($1);
    for(i=0; i<readPtr.innerSize(); i++)
    {
        PyObject *locRow = PyList_New(0);
        for(j=0; j<readPtr.outerSize(); j++)
        {
            PyObject *outObject = PyFloat_FromDouble((readPtr)(i,j));
            PyList_Append(locRow, outObject);
        }
        PyList_Append($result, locRow);
    }
}

%enddef

//EIGEN_MAT_WRAP(const Eigen::MatrixXd)
EIGEN_MAT_WRAP(Eigen::MatrixXd)
EIGEN_MAT_WRAP(Eigen::Matrix3d)
EIGEN_MAT_WRAP(Eigen::Vector3d)
