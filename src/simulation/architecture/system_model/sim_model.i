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
%module sim_model
%{
   #include "sim_model.h"
   #include "_GeneralModuleFiles/sys_interface.h"
   #include "utilities/linearAlgebra.h"
   #include "utilities/rigidBodyKinematics.h"
   #include "utilities/orbitalMotion.h"
%}

%include "std_vector.i"
%include "std_string.i"
%include "std_set.i"
%include "std_pair.i"
%include "stdint.i"
%include "carrays.i"
%include "exception.i"
%include "cdata.i"

%array_functions(double, doubleArray);
%array_functions(long, longArray);
%array_functions(int, intArray);
%array_functions(short, shortArray);
%array_functions(bool, boolArray);
%array_functions(uint8_t, cByteArray);

// Instantiate templates used by example
namespace std {
   %template(IntVector) vector<int>;
   %template(DoubleVector) vector<double>;
   %template(MultiArray) vector<vector<double>>;
   %template(StringVector) vector<string>;
   %template(StringSet) set<string>;
   %template(intSet) set<unsigned long>;
   %template(ConstCharVector) vector<const char*>;
   %template(messsageLogVector) vector<messageLogContainer>;
   %template() std::pair<long int, long int>;
   %template() std::pair<long long int, long long int>;
   %template() std::pair<int64_t, int64_t>;
   %template(exchangeSet) std::set<std::pair<long int, long int>>;
   %template(modelPriPair) std::vector<ModelPriorityPair>;
   %template(interfaceVector) std::vector<SysInterface*>;
   %template(interfaceSingVector) std::vector<InterfaceDataExchange *>;
   %template(procSchedList) std::vector<ModelScheduleEntry>;
   %template(simProcList) std::vector<SysProcess *>;
}

%inline %{
    uint64_t getObjectAddress(void *variable) {
        return (reinterpret_cast<uint64_t> (variable));
    }
%}

%exception {
    try {
        $action
    } catch (const std::exception& e) {
        SWIG_exception(SWIG_RuntimeError, e.what());
    } catch (const std::string& e) {
        SWIG_exception(SWIG_RuntimeError, e.c_str());
    } 
}

%include "sys_model_task.h"
%include "sys_model.h"
%include "sys_process.h"
%include "sys_interface.h"
%include "../utilities/message_logger.h"
%include "../messaging/system_messaging.h"
%include "../utilities/linearAlgebra.h"
%include "../utilities/rigidBodyKinematics.h"
%include "../utilities/orbitalMotion.h"
%include "sim_model.h"
