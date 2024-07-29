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
%}

%include "std_vector.i"
%include "std_string.i"
%include "std_set.i"
%include "std_pair.i"
%include "stdint.i"
%include "carrays.i"
%include "exception.i"
%include "cdata.i"
%include "swig_eigen.i"

%array_functions(double, doubleArray);
%array_functions(long, longArray);
%array_functions(int, intArray);
%array_functions(short, shortArray);
%array_functions(bool, boolArray);
%array_functions(uint8_t, cByteArray);

// Instantiate templates used by example
namespace std {
   %template(IntVector) vector<int, allocator<int> >;
   %template(DoubleVector) vector<double, allocator<double> >;
   %template(MultiArray) vector<vector<double>>;
   %template(StringVector) vector<string, allocator<string> >;
   %template(StringSet) set<string>;
   %template(intSet) set<unsigned long>;
   %template(int64Set) set<long int>;
   %template(ConstCharVector) vector<const char*, allocator<const char*> >;
   %template() std::pair<long int, long int>;
   %template() std::pair<long long int, long long int>;
   %template() std::pair<int64_t, int64_t>;
   %template(exchangeSet) set<pair<long int, long int>>;
   %template(modelPriPair) vector<ModelPriorityPair, allocator<ModelPriorityPair> >;
   %template(procSchedList) vector<ModelScheduleEntry, allocator<ModelScheduleEntry> >;
   %template(simProcList) vector<SysProcess *, allocator<SysProcess *> >;
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
%include "sim_model.h"

%pythoncode %{
    from Basilisk.utilities import deprecated
%}

%extend SimModel{
    %pythoncode %{

        @property
        def CurrentNanos(self):
            deprecated.deprecationWarn(
                    "CurrentNanos",
                    "2025/08/01",
                    "Using CurrentNanos is deprecated. Use: getCurrentNanos()\n"
            )
            return self.getCurrentNanos()

        @property
        def NextTaskTime(self):
            deprecated.deprecationWarn(
                    "NextTaskTime",
                    "2025/08/01",
                    "Using NextTaskTime is deprecated. Use: getNextTaskTime()\n"
            )
            return self.getNextTaskTime()

        def getNextTime(self):
            deprecated.deprecationWarn(
                    "getNextTime()",
                    "2025/08/01",
                    "Using getNextTime() is deprecated. Use: getNextTaskTime()\n"
            )
            return self.getNextTaskTime()

        @property
        def nextTaskTime(self):
            deprecated.deprecationWarn(
                    "nextTaskTime",
                    "2025/08/01",
                    "Using nextTaskTime is deprecated. Use: getNextTaskTime()\n"
            )
            return self.getNextTaskTime()

        @property
        def prevRouteTime(self):
            deprecated.deprecationWarn(
                    "prevRouteTime",
                    "2025/08/01",
                    "Using prevRouteTime is deprecated. Use: getPrevRouteTime()\n"
            )
            return self.getPrevRouteTime()
    %}
}