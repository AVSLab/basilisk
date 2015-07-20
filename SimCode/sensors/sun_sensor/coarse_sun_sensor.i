%module coarse_sun_sensor
%{
   #include "coarse_sun_sensor.h"
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
%feature("copyctor");
%include "sys_model.h"
%include "coarse_sun_sensor.h"
