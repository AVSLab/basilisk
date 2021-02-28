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
%module swig_eigen

%define EIGEN_MAT_WRAP(type)

%typemap(in) type {
    #include <Eigen/Dense>

    if(!PySequence_Check($input)) {
        PyErr_SetString(PyExc_ValueError,"Expected some sort of list!  Does not appear to be that.");
        return NULL;
    }

    Py_ssize_t rowLength = 0;
    Eigen::MatrixXd matrixAssemble;
    PyObject *obj = PySequence_GetItem($input, 0);
    if(!PySequence_Check(obj)) {
        rowLength = PySequence_Length($input);
        matrixAssemble.resize(PySequence_Length($input), 1);
        for(Py_ssize_t j=0; j<rowLength; j++)
        {
            matrixAssemble(j, 0) = PyFloat_AsDouble(PySequence_GetItem($input, j));
        }
    }
    else
    {
        for(Py_ssize_t i=0; i<PySequence_Length($input); i++){
            obj = PySequence_GetItem($input, i);
            if(!PySequence_Check(obj)) {
                printf("Row bad in matrix: %zd\n", i);
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
                printf("Row bad in matrix: %zd\n", i);
                PyErr_SetString(PyExc_ValueError,"All rows must be the same length!");
            }
            for(Py_ssize_t j=0; j<rowLength; j++)
            {
                matrixAssemble(i, j) = PyFloat_AsDouble(PySequence_GetItem(obj, j));
            }
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
    Py_ssize_t rowLength = 0;
    Eigen::MatrixXd matrixAssemble;
    for(Py_ssize_t i=0; i<PySequence_Length($input); i++){
        PyObject *obj = PySequence_GetItem($input, i);
        if(!PySequence_Check($input)) {
            printf("Row bad in matrix: %zd\n", i);
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
            printf("Row bad in matrix: %zd\n", i);
            PyErr_SetString(PyExc_ValueError,"All rows must be the same length!");
        }
        for(Py_ssize_t j=0; j<rowLength; j++)
        {
            matrixAssemble(i, j) = PyFloat_AsDouble(PySequence_GetItem(obj, j));
        }
    }
    type localConvert = matrixAssemble;
    $1 = new type();
    *$1 = localConvert;
}

%typemap(typecheck) type {
    $1 = PySequence_Check($input);
}

%typemap(typecheck) type & {
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
EIGEN_MAT_WRAP(Eigen::VectorXd)

