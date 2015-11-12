%module bore_ang_calc
%{
   #include "bore_ang_calc.h"
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
%include "bore_ang_calc.h"
