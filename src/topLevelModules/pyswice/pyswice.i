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

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
    #include "SpiceUsr.h"
    #include "SpiceZpr.h"
%}

%ignore illumg_c;
%ignore prefix_c;
%ignore ekucei_c;
%ignore ekuced_c;
%include "carrays.i"
%include "cstring.i"
%include "typemaps.i"

typedef const char ConstSpiceChar;
typedef double SpiceDouble;
typedef const double ConstSpiceDouble;
typedef int SpiceBoolean;
typedef const int ConstSpiceBoolean;

%define SPICEINT(spicetype)
%typemap(in) spicetype {
    $1 = (spicetype)PyLong_AsLong($input);
}
%typemap(out) spicetype {
    $result = PyLong_FromLong($1);
}
%enddef

SPICEINT(SpiceInt)
SPICEINT(ConstSpiceInt)

%array_functions(double, doubleArray);
%array_functions(SpiceInt, spiceIntArray);
%array_functions(SpiceBoolean, spiceBoolArray);

%define SPICE2DARRAYMAP(spicetype)
%typemap(in) spicetype [ANY][ANY] {
    void *dataPtr;
    SWIG_ConvertPtr($input, &dataPtr, $descriptor(spicetype *), 0 |  0);
    spicetype **actData = (spicetype**) dataPtr;
    $1 = (spicetype (*)[$1_dim1]) actData;
}
%typemap(in) spicetype [][ANY] {
    void *dataPtr;
    SWIG_ConvertPtr($input, &dataPtr, $descriptor(spicetype *), 0 |  0);
    spicetype **actData = (spicetype**) dataPtr;
    $1 = (spicetype (*)[$1_dim0]) actData;
}
%enddef

SPICE2DARRAYMAP(ConstSpiceDouble)
SPICE2DARRAYMAP(SpiceDouble)
SPICE2DARRAYMAP(ConstSpiceInt)
SPICE2DARRAYMAP(SpiceInt)


%cstring_bounded_mutable(SpiceChar *utcstr, 1024);

%include "SpiceZpr.h"
