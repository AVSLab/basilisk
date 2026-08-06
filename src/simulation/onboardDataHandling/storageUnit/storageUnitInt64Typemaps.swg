/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

%{
#include <cmath>
#include <cstdint>
#include <limits>

namespace {

bool storageUnitPyObjectToInt64(PyObject* input, int64_t* output)
{
    if (PyIndex_Check(input)) {
        PyObject* index = PyNumber_Index(input);
        if (index == nullptr) {
            return false;
        }
        const long long value = PyLong_AsLongLong(index);
        Py_DECREF(index);
        if (PyErr_Occurred()) {
            return false;
        }
        if (value < static_cast<long long>(std::numeric_limits<int64_t>::min())
            || value > static_cast<long long>(std::numeric_limits<int64_t>::max())) {
            PyErr_SetString(PyExc_OverflowError, "storage value is outside the supported int64_t range");
            return false;
        }
        *output = static_cast<int64_t>(value);
        return true;
    }

    const double value = PyFloat_AsDouble(input);
    if (PyErr_Occurred()) {
        return false;
    }
    const double int64Limit = std::ldexp(1.0, 63);
    if (!std::isfinite(value) || value < -int64Limit || value >= int64Limit) {
        PyErr_SetString(PyExc_OverflowError, "storage value is outside the supported int64_t range");
        return false;
    }
    *output = static_cast<int64_t>(value);
    return true;
}

}
%}

// Preserve exact Python integer values while continuing to accept floats such
// as 1E9 for compatibility with existing storage-unit scripts.
%typemap(in) int64_t {
    if (!storageUnitPyObjectToInt64($input, &$1)) {
        SWIG_fail;
    }
}

%typemap(in) std::vector<long long int> (std::vector<long long int> temp) {
    PyObject* sequence = PySequence_Fast($input, "expected a sequence of storage values");
    if (sequence == nullptr) {
        SWIG_fail;
    }
    const Py_ssize_t size = PySequence_Size(sequence);
    if (size < 0) {
        Py_DECREF(sequence);
        SWIG_fail;
    }
    temp.reserve(static_cast<size_t>(size));
    for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject* item = PySequence_GetItem(sequence, i);
        if (item == nullptr) {
            Py_DECREF(sequence);
            SWIG_fail;
        }
        int64_t value;
        if (!storageUnitPyObjectToInt64(item, &value)) {
            Py_DECREF(item);
            Py_DECREF(sequence);
            SWIG_fail;
        }
        Py_DECREF(item);
        temp.push_back(static_cast<long long int>(value));
    }
    Py_DECREF(sequence);
    $1 = temp;
}
