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

%include messaging.header.auto.i

%module messaging
%pythoncode %{
    from Basilisk.architecture.swig_common_model import *
%}
%include "swig_conly_data.i"
%include "std_vector.i"
%include "std_string.i"
%include "swig_eigen.i"
%include "architecture/utilities/macroDefinitions.h"
%include "fswAlgorithms/fswUtilities/fswDefinitions.h"
%include "simulation/dynamics/reactionWheels/reactionWheelSupport.h"

ARRAYASLIST(FSWdeviceAvailability)
STRUCTASLIST(CSSUnitConfigMsgPayload)
STRUCTASLIST(AccPktDataMsgPayload)
STRUCTASLIST(RWConfigElementMsgPayload)
STRUCTASLIST(CSSArraySensorMsgPayload)
STRUCTASLIST(ReconfigBurnInfoMsgPayload)

%pythoncode %{
    import numpy as np
    from Basilisk.architecture import cMsgCInterfacePy
%};
%{
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "messaging.h"
#include <vector>
%}

%template(TimeVector) std::vector<uint64_t>;
%template(DoubleVector) std::vector<double>;
%template(StringVector) std::vector<std::string>;

%include "std_vector.i"
%include "architecture/_GeneralModuleFiles/sys_model.h"
%include "messaging.h"
%rename(__subscribe_to) subscribeTo;  // we want the users to have a unified "subscribeTo" interface
%rename(__subscribe_to_C) subscribeToC;  // we want the users to have a unified "subscribeTo" interface
%rename(__is_subscribed_to) isSubscribedTo;  // we want the users to have a unified "isSubscribedTo" interface
%rename(__is_subscribed_to_C) isSubscribedToC;  // we want the users to have a unified "isSubscribedTo" interface
%rename(__time_vector) times;  // It's not really useful to give the user back a time vector
%rename(__timeWritten_vector) timesWritten;
%rename(__record_vector) record;
%define INSTANTIATE_TEMPLATES(messageType, messageTypePayload, folder)
%{
#include "architecture/folder/messageTypePayload.h"
%}
%include "architecture/folder/messageTypePayload.h"

%template(messageType ## Reader) ReadFunctor<messageTypePayload>;
%extend ReadFunctor<messageTypePayload> {
        %pythoncode %{
            def subscribeTo(self, source):
                if type(source) == messageType:
                    self.__subscribe_to(source)
                    return
                from Basilisk.architecture.cMsgCInterfacePy import messageType ## _C
                if type(source) == messageType ## _C:
                    self.__subscribe_to_C(source)
                else:
                    raise Exception('tried to subscribe ReadFunctor<messageTypePayload> to output message type' + str(type(source)))


            def isSubscribedTo(self, source):
                if type(source) == messageType:
                    return self.__is_subscribed_to(source)
                else:
                    from Basilisk.architecture.cMsgCInterfacePy import messageType ## _C
                    if type(source) == messageType ## _C:
                        return self.__is_subscribed_to_C(source)
                    else:
                        return 0                



        %}
};

%template(messageType ## Writer) WriteFunctor<messageTypePayload>;

%template(messageType) Message<messageTypePayload>;
%extend Message<messageTypePayload>{
    %pythoncode %{
        def write(self, payload, time=0, moduleID=0):
            """write the message payload.
            The 2nd argument is time in nanoseconds.  It is optional and defaults to 0.
            The 3rd argument is the module ID which defaults to 0.
            """
            writeMsg = self.addAuthor()
            writeMsg(payload, moduleID, time)  # msgs written in python have 0 module ID
            return self

        def read(self):
            """read the message payload"""
            readMsg = self.addSubscriber()
            return readMsg()
    %}
};

%template(messageType ## Recorder) Recorder<messageType ## Payload>;
%extend Recorder<messageType ## Payload> {
    %pythoncode %{
        def times(self):
            return np.array(self.__time_vector())

        def timesWritten(self):
            return np.array(self.__timeWritten_vector())

        def explore_and_find_subattr(self,attr,attr_name,content):
            if "method" in str(type(attr)) or "bool" in str(type(attr)):
                # The attribute is a method, nothing to do here
                pass
            elif isinstance(attr,list):
                # The attribute is a list of yet to be determined types
                if len(attr) > 0:
                    if "Basilisk" in str(type(attr[0])):
                        # The attribute is a list of swigged BSK objects
                        for el,k in zip(attr,range(len(attr))):
                            self.explore_and_find_subattr(el,attr_name + "[" + str(k) + "]",content)
                    else:
                        # The attribute is a list of common types 
                        content[attr_name] = attr
            elif "Basilisk" in str(type(attr)):
                # The attribute is a swigged BSK object
                # Check to see if the object is a vector and pull out the data
                if "Vector" in str(type(attr)) or "vector" in str(type(attr)):
                    content[attr_name] = []
                    for data in attr:
                        content[attr_name].append(data)
                else:
                    for subattr_name in  dir(attr):
                        if not subattr_name.startswith("__") and subattr_name != "this":
                            self.explore_and_find_subattr(getattr(attr,subattr_name),attr_name + "." + subattr_name,content)
            else:
                # The attribute has a common type
                content[attr_name] = attr


        # This __getattr__ is written in message.i.
        # It lets us return message struct attribute record as lists for plotting, etc.
        def __getattr__(self, name):
            data = self.__record_vector()
            data_record = []
            for rec in data.iterator():
                content = {}
                self.explore_and_find_subattr(rec.__getattribute__(name),name,content)
                if len(content) == 1 and "." not in str(list(content.keys())[0]):
                    data_record.append(next(iter(content.values())))
                else:
                    data_record.append(content)


            return np.array(data_record)

        def record(self):
            return self.__record_vector
    %}
};

typedef struct messageType;

%template(messageType ## PayloadVector) std::vector<messageType ## Payload>;
%extend std::vector<messageType ## Payload>{
    %pythoncode %{
        # This __getattr__ is written in message.i.
        # It lets us return message struct attribute record as lists for plotting, etc.
        def __getattr__(self, name):
            data_record = []
            for rec in self.iterator():
                data_record.append(rec.__getattribute__(name))
            return np.array(data_record)
    %}
};
%enddef

%include messaging.auto.i

%array_functions(THRConfigMsgPayload, ThrustConfigArray);
%array_functions(RWConfigElementMsgPayload, RWConfigArray);

%template(Eigen3dVector) std::vector<Eigen::Vector3d>;

%template(AccDataMsgPayloadVector) std::vector<Message<AccDataMsgPayload>*>;
%template(RWConfigLogOutMsgsVector) std::vector<Message<RWConfigLogMsgPayload>*>;
%template(SpicePlanetStateOutMsgsVector) std::vector<Message<SpicePlanetStateMsgPayload>*>;
%template(AtmoPropsOutMsgsVector) std::vector<Message<AtmoPropsMsgPayload>*>;
%template(SCStatesOutMsgsVector) std::vector<Message<SCStatesMsgPayload>*>;
%template(HingedRigidBodyOutMsgsVector) std::vector<Message<HingedRigidBodyMsgPayload>*>;
%template(THROutputOutMsgsVector) std::vector<Message<THROutputMsgPayload>*>;
%template(VSCMGConfigOutMsgsVector) std::vector<Message<VSCMGConfigMsgPayload>*>;
%template(EclipseOutMsgsVector) std::vector<Message<EclipseMsgPayload>*>;
%template(EphemerisOutMsgsVector) std::vector<Message<EphemerisMsgPayload>*>;
%template(AccessOutMsgsVector) std::vector<Message<AccessMsgPayload>*>;
%template(MagneticFieldOutMsgsVector) std::vector<Message<MagneticFieldMsgPayload>*>;
%template(AlbedoOutMsgsVector) std::vector<Message<AlbedoMsgPayload>*>;
%template(ExtTorqueOutMsgsVector) std::vector<Message<CmdTorqueBodyMsgPayload>*>;
%template(ExtBodyForceOutMsgsVector) std::vector<Message<CmdForceBodyMsgPayload>*>;
%template(ExtInertialForceOutMsgsVector) std::vector<Message<CmdForceInertialMsgPayload>*>;
%template(THROutputOutMsgsVectorVector) std::vector <std::vector <Message<THROutputMsgPayload>*>>;
%template(RWConfigLogOutMsgsVectorVector) std::vector <std::vector <Message<RWConfigLogMsgPayload>*>>;

%template(SCStatesInMsgsVector) std::vector<ReadFunctor<SCStatesMsgPayload>>;
%template(SpicePlanetStateInMsgsVector) std::vector<ReadFunctor<SpicePlanetStateMsgPayload>>;
%template(SwDataInMsgsVector) std::vector<ReadFunctor<SwDataMsgPayload>>;
%template(DataNodeUsageInMsgsVector) std::vector<ReadFunctor<DataNodeUsageMsgPayload>>;
%template(DataStorageStatusInMsgsVector) std::vector<ReadFunctor<DataStorageStatusMsgPayload>>;
%template(AccessInMsgsVector) std::vector<ReadFunctor<AccessMsgPayload>>;
%template(RWConfigLogInMsgsVector) std::vector<ReadFunctor<RWConfigLogMsgPayload>>;
%template(THROutputInMsgsVector) std::vector<ReadFunctor<THROutputMsgPayload>>;
%template(CSSConfigLogInMsgsVector) std::vector<ReadFunctor<CSSConfigLogMsgPayload>>;
%template(VolgateInMsgsVector) std::vector<ReadFunctor<VoltMsgPayload>>;
%template(DeviceCmdInMsgsVector) std::vector<ReadFunctor<DeviceCmdMsgPayload>>;


%include "messaging.h"

