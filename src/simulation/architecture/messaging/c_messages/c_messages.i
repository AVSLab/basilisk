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

/* All of the files in this folder (c_messages) are autocoded by the script GenCMessages.py.
The script checks for the line "INSTANTIATE_TEMPLATES" in the file message.i. This
ensures that if a c++ message is instantiated that we also have a C equivalent of that message.

If you need to edit the way that these types/functions are written,
edit the templates in /templates and run GenCMessages.py.
*/

%module c_messages
%{
#include "SpicePlanetStateSimMsg_C.h"
%}
%include "SpicePlanetStateSimMsg_C.h"
%extend SpicePlanetStateSimMsg_C {
    %pythoncode %{

    def subscribeTo(self, source):
        from Basilisk.simulation.message import SpicePlanetStateSimMsgClass
        if type(source) == type(self):
            SpicePlanetStateSimMsg_C_subscribe(self, source)
        elif type(source) == SpicePlanetStateSimMsgClass:
            SpicePlanetStateSimMsg_cpp_subscribe(self, source)
        else:
            raise Exception('tried to subscribe SpicePlanetStateSimMsg to another message type')

    def log(self):
        from Basilisk.simulation.message import SpicePlanetStateSimMsgLogger
        return SpicePlanetStateSimMsgLogger(self)

    def userMessage(self, data=None):
        VehicleConfigFswMsg_C_claim(self, self)
        if data:
            VehicleConfigFswMsg_C_write(data, self)
        return self
    %}
};
%{
#include "EclipseSimMsg_C.h"
%}
%include "EclipseSimMsg_C.h"
%extend EclipseSimMsg_C {
    %pythoncode %{

    def subscribeTo(self, source):
        from Basilisk.simulation.message import EclipseSimMsgClass
        if type(source) == type(self):
            EclipseSimMsg_C_subscribe(self, source)
        elif type(source) == EclipseSimMsgClass:
            EclipseSimMsg_cpp_subscribe(self, source)
        else:
            raise Exception('tried to subscribe EclipseSimMsg to another message type')

    def log(self):
        from Basilisk.simulation.message import EclipseSimMsgLogger
        return EclipseSimMsgLogger(self)

    def userMessage(self, data=None):
        VehicleConfigFswMsg_C_claim(self, self)
        if data:
            VehicleConfigFswMsg_C_write(data, self)
        return self
    %}
};
%{
#include "SCPlusStatesSimMsg_C.h"
%}
%include "SCPlusStatesSimMsg_C.h"
%extend SCPlusStatesSimMsg_C {
    %pythoncode %{

    def subscribeTo(self, source):
        from Basilisk.simulation.message import SCPlusStatesSimMsgClass
        if type(source) == type(self):
            SCPlusStatesSimMsg_C_subscribe(self, source)
        elif type(source) == SCPlusStatesSimMsgClass:
            SCPlusStatesSimMsg_cpp_subscribe(self, source)
        else:
            raise Exception('tried to subscribe SCPlusStatesSimMsg to another message type')

    def log(self):
        from Basilisk.simulation.message import SCPlusStatesSimMsgLogger
        return SCPlusStatesSimMsgLogger(self)

    def userMessage(self, data=None):
        VehicleConfigFswMsg_C_claim(self, self)
        if data:
            VehicleConfigFswMsg_C_write(data, self)
        return self
    %}
};
%{
#include "SCPlusMassPropsSimMsg_C.h"
%}
%include "SCPlusMassPropsSimMsg_C.h"
%extend SCPlusMassPropsSimMsg_C {
    %pythoncode %{

    def subscribeTo(self, source):
        from Basilisk.simulation.message import SCPlusMassPropsSimMsgClass
        if type(source) == type(self):
            SCPlusMassPropsSimMsg_C_subscribe(self, source)
        elif type(source) == SCPlusMassPropsSimMsgClass:
            SCPlusMassPropsSimMsg_cpp_subscribe(self, source)
        else:
            raise Exception('tried to subscribe SCPlusMassPropsSimMsg to another message type')

    def log(self):
        from Basilisk.simulation.message import SCPlusMassPropsSimMsgLogger
        return SCPlusMassPropsSimMsgLogger(self)

    def userMessage(self, data=None):
        VehicleConfigFswMsg_C_claim(self, self)
        if data:
            VehicleConfigFswMsg_C_write(data, self)
        return self
    %}
};
%{
#include "EpochSimMsg_C.h"
%}
%include "EpochSimMsg_C.h"
%extend EpochSimMsg_C {
    %pythoncode %{

    def subscribeTo(self, source):
        from Basilisk.simulation.message import EpochSimMsgClass
        if type(source) == type(self):
            EpochSimMsg_C_subscribe(self, source)
        elif type(source) == EpochSimMsgClass:
            EpochSimMsg_cpp_subscribe(self, source)
        else:
            raise Exception('tried to subscribe EpochSimMsg to another message type')

    def log(self):
        from Basilisk.simulation.message import EpochSimMsgLogger
        return EpochSimMsgLogger(self)

    def userMessage(self, data=None):
        VehicleConfigFswMsg_C_claim(self, self)
        if data:
            VehicleConfigFswMsg_C_write(data, self)
        return self
    %}
};
%{
#include "SpiceTimeSimMsg_C.h"
%}
%include "SpiceTimeSimMsg_C.h"
%extend SpiceTimeSimMsg_C {
    %pythoncode %{

    def subscribeTo(self, source):
        from Basilisk.simulation.message import SpiceTimeSimMsgClass
        if type(source) == type(self):
            SpiceTimeSimMsg_C_subscribe(self, source)
        elif type(source) == SpiceTimeSimMsgClass:
            SpiceTimeSimMsg_cpp_subscribe(self, source)
        else:
            raise Exception('tried to subscribe SpiceTimeSimMsg to another message type')

    def log(self):
        from Basilisk.simulation.message import SpiceTimeSimMsgLogger
        return SpiceTimeSimMsgLogger(self)

    def userMessage(self, data=None):
        VehicleConfigFswMsg_C_claim(self, self)
        if data:
            VehicleConfigFswMsg_C_write(data, self)
        return self
    %}
};
%{
#include "NavAttIntMsg_C.h"
%}
%include "NavAttIntMsg_C.h"
%extend NavAttIntMsg_C {
    %pythoncode %{

    def subscribeTo(self, source):
        from Basilisk.simulation.message import NavAttIntMsgClass
        if type(source) == type(self):
            NavAttIntMsg_C_subscribe(self, source)
        elif type(source) == NavAttIntMsgClass:
            NavAttIntMsg_cpp_subscribe(self, source)
        else:
            raise Exception('tried to subscribe NavAttIntMsg to another message type')

    def log(self):
        from Basilisk.simulation.message import NavAttIntMsgLogger
        return NavAttIntMsgLogger(self)

    def userMessage(self, data=None):
        VehicleConfigFswMsg_C_claim(self, self)
        if data:
            VehicleConfigFswMsg_C_write(data, self)
        return self
    %}
};
%{
#include "NavTransIntMsg_C.h"
%}
%include "NavTransIntMsg_C.h"
%extend NavTransIntMsg_C {
    %pythoncode %{

    def subscribeTo(self, source):
        from Basilisk.simulation.message import NavTransIntMsgClass
        if type(source) == type(self):
            NavTransIntMsg_C_subscribe(self, source)
        elif type(source) == NavTransIntMsgClass:
            NavTransIntMsg_cpp_subscribe(self, source)
        else:
            raise Exception('tried to subscribe NavTransIntMsg to another message type')

    def log(self):
        from Basilisk.simulation.message import NavTransIntMsgLogger
        return NavTransIntMsgLogger(self)

    def userMessage(self, data=None):
        VehicleConfigFswMsg_C_claim(self, self)
        if data:
            VehicleConfigFswMsg_C_write(data, self)
        return self
    %}
};
%{
#include "CmdTorqueBodyIntMsg_C.h"
%}
%include "CmdTorqueBodyIntMsg_C.h"
%extend CmdTorqueBodyIntMsg_C {
    %pythoncode %{

    def subscribeTo(self, source):
        from Basilisk.simulation.message import CmdTorqueBodyIntMsgClass
        if type(source) == type(self):
            CmdTorqueBodyIntMsg_C_subscribe(self, source)
        elif type(source) == CmdTorqueBodyIntMsgClass:
            CmdTorqueBodyIntMsg_cpp_subscribe(self, source)
        else:
            raise Exception('tried to subscribe CmdTorqueBodyIntMsg to another message type')

    def log(self):
        from Basilisk.simulation.message import CmdTorqueBodyIntMsgLogger
        return CmdTorqueBodyIntMsgLogger(self)

    def userMessage(self, data=None):
        VehicleConfigFswMsg_C_claim(self, self)
        if data:
            VehicleConfigFswMsg_C_write(data, self)
        return self
    %}
};
%{
#include "CmdForceBodyIntMsg_C.h"
%}
%include "CmdForceBodyIntMsg_C.h"
%extend CmdForceBodyIntMsg_C {
    %pythoncode %{

    def subscribeTo(self, source):
        from Basilisk.simulation.message import CmdForceBodyIntMsgClass
        if type(source) == type(self):
            CmdForceBodyIntMsg_C_subscribe(self, source)
        elif type(source) == CmdForceBodyIntMsgClass:
            CmdForceBodyIntMsg_cpp_subscribe(self, source)
        else:
            raise Exception('tried to subscribe CmdForceBodyIntMsg to another message type')

    def log(self):
        from Basilisk.simulation.message import CmdForceBodyIntMsgLogger
        return CmdForceBodyIntMsgLogger(self)

    def userMessage(self, data=None):
        VehicleConfigFswMsg_C_claim(self, self)
        if data:
            VehicleConfigFswMsg_C_write(data, self)
        return self
    %}
};
%{
#include "CmdForceInertialIntMsg_C.h"
%}
%include "CmdForceInertialIntMsg_C.h"
%extend CmdForceInertialIntMsg_C {
    %pythoncode %{

    def subscribeTo(self, source):
        from Basilisk.simulation.message import CmdForceInertialIntMsgClass
        if type(source) == type(self):
            CmdForceInertialIntMsg_C_subscribe(self, source)
        elif type(source) == CmdForceInertialIntMsgClass:
            CmdForceInertialIntMsg_cpp_subscribe(self, source)
        else:
            raise Exception('tried to subscribe CmdForceInertialIntMsg to another message type')

    def log(self):
        from Basilisk.simulation.message import CmdForceInertialIntMsgLogger
        return CmdForceInertialIntMsgLogger(self)

    def userMessage(self, data=None):
        VehicleConfigFswMsg_C_claim(self, self)
        if data:
            VehicleConfigFswMsg_C_write(data, self)
        return self
    %}
};
%{
#include "EphemerisIntMsg_C.h"
%}
%include "EphemerisIntMsg_C.h"
%extend EphemerisIntMsg_C {
    %pythoncode %{

    def subscribeTo(self, source):
        from Basilisk.simulation.message import EphemerisIntMsgClass
        if type(source) == type(self):
            EphemerisIntMsg_C_subscribe(self, source)
        elif type(source) == EphemerisIntMsgClass:
            EphemerisIntMsg_cpp_subscribe(self, source)
        else:
            raise Exception('tried to subscribe EphemerisIntMsg to another message type')

    def log(self):
        from Basilisk.simulation.message import EphemerisIntMsgLogger
        return EphemerisIntMsgLogger(self)

    def userMessage(self, data=None):
        VehicleConfigFswMsg_C_claim(self, self)
        if data:
            VehicleConfigFswMsg_C_write(data, self)
        return self
    %}
};
%{
#include "AttRefFswMsg_C.h"
%}
%include "AttRefFswMsg_C.h"
%extend AttRefFswMsg_C {
    %pythoncode %{

    def subscribeTo(self, source):
        from Basilisk.simulation.message import AttRefFswMsgClass
        if type(source) == type(self):
            AttRefFswMsg_C_subscribe(self, source)
        elif type(source) == AttRefFswMsgClass:
            AttRefFswMsg_cpp_subscribe(self, source)
        else:
            raise Exception('tried to subscribe AttRefFswMsg to another message type')

    def log(self):
        from Basilisk.simulation.message import AttRefFswMsgLogger
        return AttRefFswMsgLogger(self)

    def userMessage(self, data=None):
        VehicleConfigFswMsg_C_claim(self, self)
        if data:
            VehicleConfigFswMsg_C_write(data, self)
        return self
    %}
};
%{
#include "AttGuidFswMsg_C.h"
%}
%include "AttGuidFswMsg_C.h"
%extend AttGuidFswMsg_C {
    %pythoncode %{

    def subscribeTo(self, source):
        from Basilisk.simulation.message import AttGuidFswMsgClass
        if type(source) == type(self):
            AttGuidFswMsg_C_subscribe(self, source)
        elif type(source) == AttGuidFswMsgClass:
            AttGuidFswMsg_cpp_subscribe(self, source)
        else:
            raise Exception('tried to subscribe AttGuidFswMsg to another message type')

    def log(self):
        from Basilisk.simulation.message import AttGuidFswMsgLogger
        return AttGuidFswMsgLogger(self)

    def userMessage(self, data=None):
        VehicleConfigFswMsg_C_claim(self, self)
        if data:
            VehicleConfigFswMsg_C_write(data, self)
        return self
    %}
};
%{
#include "VehicleConfigFswMsg_C.h"
%}
%include "VehicleConfigFswMsg_C.h"
%extend VehicleConfigFswMsg_C {
    %pythoncode %{

    def subscribeTo(self, source):
        from Basilisk.simulation.message import VehicleConfigFswMsgClass
        if type(source) == type(self):
            VehicleConfigFswMsg_C_subscribe(self, source)
        elif type(source) == VehicleConfigFswMsgClass:
            VehicleConfigFswMsg_cpp_subscribe(self, source)
        else:
            raise Exception('tried to subscribe VehicleConfigFswMsg to another message type')

    def log(self):
        from Basilisk.simulation.message import VehicleConfigFswMsgLogger
        return VehicleConfigFswMsgLogger(self)

    def userMessage(self, data=None):
        VehicleConfigFswMsg_C_claim(self, self)
        if data:
            VehicleConfigFswMsg_C_write(data, self)
        return self
    %}
};
