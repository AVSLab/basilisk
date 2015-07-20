%module thruster_dynamics
%{
   #include "thruster_dynamics.h"
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
   %template(ThrusterTimeVector) vector<ThrusterTimePair>;
   %template(ThrusterConfigVector) vector<ThrusterConfigData>;
}
%include "sys_model.h"
%include "dyn_effector.h"
%include "thruster_dynamics.h"
