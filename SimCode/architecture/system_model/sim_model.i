%module sim_model
%{
   #include "sim_model.h"
   #include "sys_interface.h"
   #include "utilities/linearAlgebra.h"
   #include "utilities/rigidBodyKinematics.h"
%}

%include "std_vector.i"
%include "std_string.i"
%include "std_set.i"
%include "stdint.i"
%include "carrays.i"

%array_functions(double, doubleArray);
%array_functions(long, longArray);
%array_functions(int, intArray);
%array_functions(short, shortArray);
%array_functions(bool, boolArray);

// Instantiate templates used by example
namespace std {
   %template(IntVector) vector<int>;
   %template(DoubleVector) vector<double>;
   %template(StringVector) vector<string>;
   %template(StringSet) set<string>;
   %template(ConstCharVector) vector<const char*>;
   %template(messsageLogVector) vector<messageLogContainer>;
}
%include "sys_model_task.h"
%include "sys_process.h"
%include "sys_interface.h"
%include "message_logger.h"
%include "../messaging/system_messaging.h"
%include "linearAlgebra.h"
%include "rigidBodyKinematics.h"
%include "sim_model.h"
