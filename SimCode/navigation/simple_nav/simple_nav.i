%module simple_nav
%{
   #include "simple_nav.h"
%}

%include "std_vector.i"
%include "std_string.i"
%include "std_map.i"
%include "stdint.i"

// Instantiate templates used by example
namespace std {
   %template(IntVector) vector<int>;
   %template(DoubleVector) vector<double>;
   %template(StringVector) vector<string>;
   %template(ConstCharVector) vector<const char*>;
}
%include "sys_model.h"
%include "simple_nav.h"
%include "../../ADCSAlgorithms/attDetermination/CSSEst/navStateOut.h"
