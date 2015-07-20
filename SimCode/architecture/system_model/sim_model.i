%module sim_model
%{
   #include "sim_model.h"
%}

%include "std_vector.i"
%include "std_string.i"
%include "stdint.i"

// Instantiate templates used by example
namespace std {
   %template(IntVector) vector<int>;
   %template(DoubleVector) vector<double>;
   %template(StringVector) vector<string>;
   %template(ConstCharVector) vector<const char*>;
}
%include "sys_model_thread.h"
%include "sim_model.h"
