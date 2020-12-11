/*
Copyright (c) 2020, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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

%include messaging2.header.auto.i

%module messaging2
%pythoncode %{
    from Basilisk.simulation.swig_common_model import *
%}
%include "swig_conly_data.i"
%include "std_vector.i"
%include "std_string.i"
%include "swig_eigen.i"
%include "utilities/macroDefinitions.h"
%include "fswUtilities/fswDefinitions.h"
%include "dynamics/reactionWheels/reactionWheelSupport.h"

ARRAYASLIST(FSWdeviceAvailability)
STRUCTASLIST(CSSUnitConfigMsgPayload)
STRUCTASLIST(AccPktDataMsgPayload)
STRUCTASLIST(RWConfigElementMsgPayload)
STRUCTASLIST(CSSArraySensorMsgPayload)


%pythoncode %{
    import numpy as np
    from Basilisk.simulation import cMsgCInterfacePy
%};
%{
#include "_GeneralModuleFiles/sys_model.h"
#include "messaging2.h"
#include <vector>
%}
%template(TimeVector) std::vector<uint64_t>;
%include "std_vector.i"
%include "_GeneralModuleFiles/sys_model.h"
%include "messaging2.h"
%rename(__subscribe_to) subscribeTo;  // we want the users to have a unified "subscribeTo" interface
%rename(__subscribe_to_C) subscribeToC;  // we want the users to have a unified "subscribeTo" interface
%rename(__time_vector) times;  // It's not really useful to give the user back a time vector
%rename(__record_vector) record;
%define INSTANTIATE_TEMPLATES(messageType, messageTypePayload, folder)
%{
#include "folder/messageTypePayload.h"
%}
%include "folder/messageTypePayload.h"

%template(messageType ## Reader) ReadFunctor<messageTypePayload>;
%extend ReadFunctor<messageTypePayload> {
        %pythoncode %{
            def subscribeTo(self, source):
                from Basilisk.simulation.cMsgCInterfacePy import messageType ## _C
                if type(source) == messageType:
                    self.__subscribe_to(source)
                elif type(source) == messageType ## _C:
                    self.__subscribe_to_C(source)
                else:
                    raise Exception('tried to subscribe ReadFunctor<messageTypePayload> to output message type' + str(type(source)))
        %}
};

%template(messageType ## Writer) WriteFunctor<messageTypePayload>;

%template(messageType) Message<messageTypePayload>;
%extend Message<messageTypePayload>{
    %pythoncode %{
        def write(self, payload, time=0):
            """write the message payload.  The second argument is time in nanoseconds.  It is optional and defaults to 0."""
            writeMsg = self.addAuthor()
            writeMsg(payload, -1, time)  # msgs written in python have -1 module ID
            return self

        def read(self):
            """read the message payload"""
            readMsg = self.addSubscriber()
            return readMsg()
    %}
};

%template(messageType ## Logger) Log<messageType ## Payload>;
%extend Log<messageType ## Payload> {
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

%template(messageType ## Vector) std::vector<messageType ## Payload>;
%extend std::vector<messageType ## Payload>{
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

%include messaging2.auto.i

%array_functions(THRConfigMsgPayload, ThrustConfigArray);
%array_functions(RWConfigElementMsgPayload, RWConfigArray);

%template(RWConfigOutMsgVector) std::vector<Message<RWConfigLogMsgPayload>>;

%include "messaging2.h"
