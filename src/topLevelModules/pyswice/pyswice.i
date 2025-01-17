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
%array_functions(double, doubleArray);
%array_functions(int, intArray);

// NOTE: These typedefs are for the C code
//       These should be defined by SpiceUsr.h but may not be on some platforms
%{
    typedef const char ConstSpiceChar;
    typedef double SpiceDouble;
    typedef const double ConstSpiceDouble;
    typedef int SpiceInt;
    typedef const int ConstSpiceInt;
    typedef int SpiceBoolean;
%}
// NOTE: These typedefs are for SWIG generation
typedef const char ConstSpiceChar;
typedef double SpiceDouble;
typedef const double ConstSpiceDouble;
typedef int SpiceInt;
typedef const int ConstSpiceInt;
typedef int SpiceBoolean;

%define SPICE2DARRAYMAP(ctype, spicetype)
%typemap(in) spicetype [ANY][ANY] {
    void *dataPtr;
    SWIG_ConvertPtr($input, &dataPtr, $descriptor(spicetype *), 0 |  0);
    ctype **actData = (ctype**) dataPtr;
    $1 = (spicetype (*)[$1_dim1]) actData;
}
%typemap(in) spicetype [][ANY] {
    void *dataPtr;
    SWIG_ConvertPtr($input, &dataPtr, $descriptor(spicetype *), 0 |  0);
    ctype **actData = (ctype**) dataPtr;
    $1 = (spicetype (*)[$1_dim0]) actData;
}
%enddef

SPICE2DARRAYMAP(double, ConstSpiceDouble)
SPICE2DARRAYMAP(double, SpiceDouble)
SPICE2DARRAYMAP(int, ConstSpiceInt)
SPICE2DARRAYMAP(int, SpiceInt)


%cstring_bounded_mutable(SpiceChar *utcstr, 1024);

%include "SpiceZpr.h"
