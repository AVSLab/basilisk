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
%module sys_model_task
%{
   #include "sys_model_task.h"
%}

%include "std_vector.i"
%include "std_string.i"
%include "stdint.i"

// Instantiate templates used by example
namespace std {
   %template(IntVector) vector<int, allocator<int> >;
   %template(DoubleVector) vector<double, allocator<double> >;
   %template(StringVector) vector<string, allocator<string> >;
   %template(ConstCharVector) vector<const char*, allocator<const char*> >;
}
%include "sys_model.h"
%include "sys_model_task.h"

%pythoncode %{
    from Basilisk.utilities import deprecated
%}

%extend SysModelTask {
    %pythoncode %{

    @property
    def NextStartTime(self):
        deprecated.deprecationWarn(
            "NextStartTime",
            "2025/08/01",
            "Using NextStartTime is deprecated. Use: getNextStartTime()\n"
        )
        return self.getNextStartTime()

    @NextStartTime.setter
    def NextStartTime(self, value):
        deprecated.deprecationWarn(
            "NextStartTime",
            "2025/08/01",
            "Using NextStartTime is deprecated. Use: setNextStartTime()\n"
        )
        self.setNextStartTime(value)

    @property
    def TaskPeriod(self):
        deprecated.deprecationWarn(
            "TaskPeriod",
            "2025/08/01",
            "Using TaskPeriod is deprecated. Use: getTaskPeriod()\n"
        )
        return self.getTaskPeriod()

    @TaskPeriod.setter
    def TaskPeriod(self, value):
        deprecated.deprecationWarn(
            "TaskPeriod",
            "2025/08/01",
            "Using TaskPeriod is deprecated. Use: setTaskPeriod()\n"
        )
        self.setTaskPeriod(value)

    @property
    def NextPickupTime(self):
        deprecated.deprecationWarn(
            "NextPickupTime",
            "2025/08/01",
            "Using NextPickupTime is deprecated. Use: getNextPickupTime()\n"
        )
        return self.getNextPickupTime()

    @NextPickupTime.setter
    def NextPickupTime(self, value):
        deprecated.deprecationWarn(
            "NextPickupTime",
            "2025/08/01",
            "Using NextPickupTime is deprecated. Use: setNextPickupTime()\n"
        )
        self.setNextPickupTime(value)

    @property
    def FirstTaskTime(self):
        deprecated.deprecationWarn(
            "FirstTaskTime",
            "2025/08/01",
            "Using FirstTaskTime is deprecated. Use: getFirstTaskTime()\n"
        )
        return self.getFirstTaskTime()

    @FirstTaskTime.setter
    def FirstTaskTime(self, value):
        deprecated.deprecationWarn(
            "FirstTaskTime",
            "2025/08/01",
            "Using FirstTaskTime is deprecated. Use: setFirstTaskTime()\n"
        )
        self.setFirstTaskTime(value)
    %}
}