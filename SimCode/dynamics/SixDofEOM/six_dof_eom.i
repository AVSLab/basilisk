%module six_dof_eom
%{
   #include "six_dof_eom.h"
%}

%include "std_vector.i"
%include "std_string.i"
%include "std_map.i"
%include "stdint.i"
%include "carrays.i"

// Instantiate templates used by example
namespace std {
   %template(IntVector) vector<int>;
   %template(DoubleVector) vector<double>;
   %template(StringVector) vector<string>;
   %template(ConstCharVector) vector<const char*>;
   %template(GravityBodyDataVector) vector<GravityBodyData>;
}

%include "../utilities/coeffLoader.h"
%include "../utilities/sphericalHarmonics.h"

%include "sys_model.h"
%include "dyn_effector.h"
%include "six_dof_eom.h"
