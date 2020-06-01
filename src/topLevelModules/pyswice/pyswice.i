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

 
%module pyswice
%{
    #include "../libs/cspice/include/SpiceUsr.h"
    #include "../libs/cspice/include/SpiceZpr.h"
%}

%ignore illumg_c;
%ignore prefix_c;
%ignore ekucei_c;
%ignore ekuced_c;
%include "carrays.i"
%include "cstring.i"
%include "typemaps.i"
%array_functions(double, doubleArray);
%array_functions(int, intArray);
typedef char ConstSpiceChar;
typedef double SpiceDouble;
%typemap(in) ConstSpiceDouble[][4] {
    void *dataPtr;
    res9 = SWIG_ConvertPtr($input, &dataPtr, $descriptor(ConstSpiceDouble *), 0 |  0);
    double **actData = (double**) dataPtr;
    $1 = (ConstSpiceDouble (*)[4]) actData;
}
%typemap(in) ConstSpiceDouble[][3] {
    void *dataPtr;
    res9 = SWIG_ConvertPtr($input, &dataPtr, $descriptor(ConstSpiceDouble *), 0 |  0);
    double **actData = (double**) dataPtr;
    $1 = (ConstSpiceDouble (*)[3]) actData;
}
%typemap(in) SpiceDouble[3][3] {
    void *dataPtr;
    SWIG_ConvertPtr($input, &dataPtr, $descriptor(ConstSpiceDouble *), 0 |  0);
    double **actData = (double**) dataPtr;
    $1 = (SpiceDouble (*)[3]) actData;
}
%typemap(in) SpiceDouble[6][6] {
    void *dataPtr;
    SWIG_ConvertPtr($input, &dataPtr, $descriptor(ConstSpiceDouble *), 0 |  0);
    double **actData = (double**) dataPtr;
    $1 = (SpiceDouble (*)[6]) actData;
}
typedef double ConstSpiceDouble;
typedef double ConstSpiceDouble;
typedef int SpiceInt;
typedef int SpiceBoolean;


%cstring_bounded_mutable(SpiceChar *utcstr, 1024);

%include "../../libs/cspice/include/SpiceZpr.h"

