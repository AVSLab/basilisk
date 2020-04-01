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
#define SWIGPYTHON_BUILTIN

#ifdef __APPLE__
#else
#define SWIGWORDSIZE64
#endif

%include "swig_common_model.i"

%module message
%pythoncode %{
    import numpy as np
%};
%{
#include "../../_GeneralModuleFiles/sys_model.h"
#include "message.h"
#include <vector>
%}
%template(TimeVector) std::vector<uint64_t>;
%include "std_vector.i"
%include "../../_GeneralModuleFiles/sys_model.h"
%include "message.h"
%rename(__subscribe_to) subscribeTo;  // we want the users to have a unified "subscribeTo" interface
%rename(__subscribe_to_C) subscribeToC;  // we want the users to have a unified "subscribeTo" interface
%define INSTANTIATE_TEMPLATES(messageType,message_file, directory)
%{
#include "../../directory/message_file.h"
%}
%include "../../directory/message_file.h"

%template(messageType ## Reader) ReadFunctor<messageType>;
%extend ReadFunctor<messageType> {
        %pythoncode %{
            def subscribeTo(self, source):
                from Basilisk.simulation.c_messages import messageType ## _C
                if type(source) == messageType ## Class:
                    self.__subscribe_to(source)
                elif type(source) == messageType ## _C:
                    self.__subscribe_to_C(source)
                else:
                    raise Exception('tried to subscribe ReadFunctor<messageType> to another message type')
        %}
};

%template(messageType ## Writer) WriteFunctor<messageType>;

%template(messageType ## Class) SimMessage<messageType>;

%template(messageType ## Logger) Log<messageType>;
%rename(__time_vector) times;  // It's not really useful to give the user back a time vector
%rename(__record_vector) record;
%extend Log<messageType> {
    %pythoncode %{
        def times(self):
            return np.array(self.__time_vector())

        # This __getattr__ is written in message.i.
        # It lets us return message struct attribute record as lists for plotting, etc.
        def __getattr__(self, name):
            data = self.__record_vector()
            data_log = []
            for rec in data.iterator():
                data_log.append(rec.__getattribute__(name))
            return np.array(data_log)

        def record(self):
            return self.__record_vector
    %}
};

typedef struct messageType;

%template(messageType ## Vector) std::vector<messageType>;
%extend std::vector<messageType>{
    %pythoncode %{
        # This __getattr__ is written in message.i.
        # It lets us return message struct attribute record as lists for plotting, etc.
        def __getattr__(self, name):
            data_log = []
            for rec in self.iterator():
                data_log.append(rec.__getattribute__(name))
            return np.array(data_log)
    %}
};
%enddef

//sim messages
INSTANTIATE_TEMPLATES(SpicePlanetStateSimMsg, spicePlanetStateSimMsg, simMessages)
INSTANTIATE_TEMPLATES(EclipseSimMsg, eclipseSimMsg, simMessages)
INSTANTIATE_TEMPLATES(SCPlusStatesSimMsg, scPlusStatesSimMsg, simMessages)
INSTANTIATE_TEMPLATES(SCPlusMassPropsSimMsg, scPlusMassPropsSimMsg, simMessages)
INSTANTIATE_TEMPLATES(EpochSimMsg, epochSimMsg, simMessages)
INSTANTIATE_TEMPLATES(SpiceTimeSimMsg, spiceTimeSimMsg, simMessages)

//fsw interface messages
INSTANTIATE_TEMPLATES(NavAttIntMsg, navAttIntMsg, simFswInterfaceMessages)
INSTANTIATE_TEMPLATES(NavTransIntMsg, navTransIntMsg, simFswInterfaceMessages)
INSTANTIATE_TEMPLATES(CmdTorqueBodyIntMsg, cmdTorqueBodyIntMsg, simMessages)
INSTANTIATE_TEMPLATES(CmdForceBodyIntMsg, cmdForceBodyIntMsg, simFswInterfaceMessages)
INSTANTIATE_TEMPLATES(CmdForceInertialIntMsg, cmdForceInertialIntMsg, simFswInterfaceMessages)
INSTANTIATE_TEMPLATES(EphemerisIntMsg, ephemerisIntMsg, simFswInterfaceMessages)

//fsw messages
INSTANTIATE_TEMPLATES(AttRefFswMsg, attRefFswMsg, ../fswAlgorithms/fswMessages)
INSTANTIATE_TEMPLATES(AttGuidFswMsg, attGuidFswMsg, ../fswAlgorithms/fswMessages)
INSTANTIATE_TEMPLATES(VehicleConfigFswMsg, vehicleConfigFswMsg, ../fswAlgorithms/fswMessages)

%include "message.h"
