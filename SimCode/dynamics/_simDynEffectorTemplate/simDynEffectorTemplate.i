%module simDynEffectorTemplate
%{
   #include "simDynEffectorTemplate.h"
%}

// SWIG translates basic C/C++ type variables (bool, int, double etc.)
// directly to the matching variable type in Python. However, more complex
// templated types from the standard library like std:vector<> require
// additional SWIG libraries. Immediately below we include the SWIG libraries
// for std::vector, std::string, std::map and std::int
%include "std_vector.i"
%include "std_string.i"
%include "std_map.i"
%include "stdint.i"

// Having included the additional SWIG type mapping libraries above
// we now use the %template directive to create instantiations of a
// C++ template class e.g. std::vector<double>.
namespace std {
   %template(IntVector) vector<int>;        // std::vector<int>
   %template(DoubleVector) vector<double>;  // std::vector<double>
   %template(StringVector) vector<string>;  // std::vector<std::string>
   %template(ConstCharVector) vector<const char*>; // std::vector<char>
}

%include "sys_model.h"
%include "dyn_effector.h"
// Replace the following include with the header from your module
%include "simDynEffectorTemplate.h"
